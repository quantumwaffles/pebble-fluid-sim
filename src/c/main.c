#include <pebble.h>

// ---------------------------------------------------------------------------
// Grid dimensions — platform-specific, all use 4px cells
// ---------------------------------------------------------------------------
// Basalt/Diorite: 144x168 -> 36x42
// Chalk:          180x180 -> 45x45
// Emery:          200x228 -> 50x57
// Aplite:         144x168 -> 36x42
#define CELL_SIZE 4

#if defined(PBL_PLATFORM_EMERY)
  #define GRID_W 50
  #define GRID_H 57
#elif defined(PBL_PLATFORM_CHALK)
  #define GRID_W 45
  #define GRID_H 45
#else
  // aplite, basalt, diorite
  #define GRID_W 36
  #define GRID_H 42
#endif

#define N_CELLS  (GRID_W * GRID_H)
// Array size matches this platform's grid exactly
#define N_CELLS_MAX N_CELLS

// Fixed-point scale for velocities
// 1 unit = 1/16 cell per step
// Aplite has very little RAM so we keep int8_t there (capped at +-127)
// All other platforms use int16_t for higher velocity range
#define VEL_SCALE 16
#if defined(PBL_PLATFORM_APLITE)
  typedef int8_t vel_t;
  #define VEL_MAX  127
  #define VEL_MIN -128
#else
  typedef int16_t vel_t;
  #define VEL_MAX  32767
  #define VEL_MIN -32768
#endif

// Diffusion
#define DIFFUSE_STEPS   1

// Timer
#define FRAME_MS        33    // ~30 fps

// ---------------------------------------------------------------------------
// Simulation state
// ---------------------------------------------------------------------------
static uint8_t  s_density[N_CELLS_MAX];
static uint8_t  s_density_prev[N_CELLS_MAX];
static vel_t    s_vx[N_CELLS_MAX];
static vel_t    s_vy[N_CELLS_MAX];
static vel_t    s_vx_prev[N_CELLS_MAX];
static vel_t    s_vy_prev[N_CELLS_MAX];

// ---------------------------------------------------------------------------
// Pebble handles
// ---------------------------------------------------------------------------
static Window       *s_window;
static Layer        *s_canvas_layer;
static AppTimer     *s_timer;

// Button hold state — set on raw press, cleared on raw release
static bool s_btn_up   = false;
static bool s_btn_down = false;
static bool s_btn_select = false;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline int idx(int x, int y) { return y * GRID_W + x; }

static inline int clamp_i(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Fast LCG random: returns -range..+range
static uint32_t s_rand = 12345;
static inline int fast_rand(int range) {
  s_rand = s_rand * 1664525u + 1013904223u;
  return (int)(s_rand >> 24) % (range * 2 + 1) - range;
}

static void clear_prev(void) {
  memset(s_density_prev, 0, sizeof(s_density_prev));
  memset(s_vx_prev,      0, sizeof(s_vx_prev));
  memset(s_vy_prev,      0, sizeof(s_vy_prev));
}

// ---------------------------------------------------------------------------
// Fluid solver
// ---------------------------------------------------------------------------

// Add source: dst += src (clamped to uint8)
static void add_source_density(uint8_t *dst, const uint8_t *src) {
  for (int i = 0; i < N_CELLS; i++) {
    int v = (int)dst[i] + (int)src[i];
    dst[i] = (uint8_t)(v > 255 ? 255 : v);
  }
}

static void add_source_vel(vel_t *dst, const vel_t *src) {
  for (int i = 0; i < N_CELLS; i++) {
    int v = (int)dst[i] + (int)src[i];
    dst[i] = (int16_t)(v > VEL_MAX ? VEL_MAX : (v < VEL_MIN ? VEL_MIN : v));
  }
}

// Simple diffuse using Gauss-Seidel relaxation (uint8 density)
static void diffuse_density(uint8_t *dst, const uint8_t *src) {
  // Weak diffusion: a = 1/16.  v = (src + a*sum) / (1 + 4a)
  // = (16*src + sum) / 20  (approximated with /16 shift below for stability)
  for (int k = 0; k < DIFFUSE_STEPS; k++) {
    for (int y = 1; y < GRID_H - 1; y++) {
      for (int x = 1; x < GRID_W - 1; x++) {
        int sum = (int)dst[idx(x-1,y)] + (int)dst[idx(x+1,y)]
                + (int)dst[idx(x,y-1)] + (int)dst[idx(x,y+1)];
        int v = (16 * (int)src[idx(x,y)] + sum) / 20;
        dst[idx(x,y)] = (uint8_t)(v > 255 ? 255 : v);
      }
    }
  }
}

// Bilinear advection for density
static void advect_density(uint8_t *dst, const uint8_t *src,
                            const vel_t *vx, const vel_t *vy) {
  // Backtrace position must stay within [1, GRID-2] in cell space so that
  // fluid hitting a wall piles up there instead of vanishing.
  const int lo_x = VEL_SCALE;                  // cell 1.0
  const int hi_x = (GRID_W - 2) * VEL_SCALE;   // cell (GRID_W-2).0
  const int lo_y = VEL_SCALE;
  const int hi_y = (GRID_H - 2) * VEL_SCALE;

  for (int y = 1; y < GRID_H - 1; y++) {
    for (int x = 1; x < GRID_W - 1; x++) {
      // trace backwards in fixed-point, clamp the POSITION (not the cell index)
      int ox = x * VEL_SCALE - (int)vx[idx(x,y)];
      int oy = y * VEL_SCALE - (int)vy[idx(x,y)];
      ox = clamp_i(ox, lo_x, hi_x);
      oy = clamp_i(oy, lo_y, hi_y);

      int x0 = ox / VEL_SCALE;
      int y0 = oy / VEL_SCALE;
      int fx = ox - x0 * VEL_SCALE;  // now always 0..VEL_SCALE-1
      int fy = oy - y0 * VEL_SCALE;
      int x1 = x0 + 1, y1 = y0 + 1;

      // bilinear interpolation
      int v = (src[idx(x0,y0)] * (VEL_SCALE - fx) * (VEL_SCALE - fy)
             + src[idx(x1,y0)] * fx               * (VEL_SCALE - fy)
             + src[idx(x0,y1)] * (VEL_SCALE - fx) * fy
             + src[idx(x1,y1)] * fx               * fy)
             / (VEL_SCALE * VEL_SCALE);

      dst[idx(x,y)] = (uint8_t)(v > 255 ? 255 : v);
    }
  }
}

// Advect velocity component
static void advect_vel(vel_t *dst, const vel_t *src,
                       const vel_t *vx, const vel_t *vy) {
  const int lo_x = VEL_SCALE;
  const int hi_x = (GRID_W - 2) * VEL_SCALE;
  const int lo_y = VEL_SCALE;
  const int hi_y = (GRID_H - 2) * VEL_SCALE;

  for (int y = 1; y < GRID_H - 1; y++) {
    for (int x = 1; x < GRID_W - 1; x++) {
      int ox = x * VEL_SCALE - (int)vx[idx(x,y)];
      int oy = y * VEL_SCALE - (int)vy[idx(x,y)];
      ox = clamp_i(ox, lo_x, hi_x);
      oy = clamp_i(oy, lo_y, hi_y);

      int x0 = ox / VEL_SCALE;
      int y0 = oy / VEL_SCALE;
      int fx = ox - x0 * VEL_SCALE;
      int fy = oy - y0 * VEL_SCALE;
      int x1 = x0 + 1, y1 = y0 + 1;

      int v = ((int)src[idx(x0,y0)] * (VEL_SCALE - fx) * (VEL_SCALE - fy)
             + (int)src[idx(x1,y0)] * fx               * (VEL_SCALE - fy)
             + (int)src[idx(x0,y1)] * (VEL_SCALE - fx) * fy
             + (int)src[idx(x1,y1)] * fx               * fy)
             / (VEL_SCALE * VEL_SCALE);

      dst[idx(x,y)] = (vel_t)(v > VEL_MAX ? VEL_MAX : (v < VEL_MIN ? VEL_MIN : v));
    }
  }
}

// Hodge projection: remove divergence from velocity field
static void project(vel_t *vx, vel_t *vy) {
  static vel_t div[N_CELLS_MAX];
  static vel_t p[N_CELLS_MAX];
  memset(p, 0, sizeof(p));

  for (int y = 1; y < GRID_H - 1; y++) {
    for (int x = 1; x < GRID_W - 1; x++) {
      div[idx(x,y)] = (vel_t)(
        ((int)vx[idx(x+1,y)] - (int)vx[idx(x-1,y)]
       + (int)vy[idx(x,y+1)] - (int)vy[idx(x,y-1)]) / 4
      );
    }
  }

  for (int k = 0; k < DIFFUSE_STEPS; k++) {
    for (int y = 1; y < GRID_H - 1; y++) {
      for (int x = 1; x < GRID_W - 1; x++) {
        int sum = (int)p[idx(x-1,y)] + (int)p[idx(x+1,y)]
                + (int)p[idx(x,y-1)] + (int)p[idx(x,y+1)];
        int v = (sum - (int)div[idx(x,y)]) / 4;
        p[idx(x,y)] = (vel_t)(v > VEL_MAX ? VEL_MAX : (v < VEL_MIN ? VEL_MIN : v));
      }
    }
  }

  for (int y = 1; y < GRID_H - 1; y++) {
    for (int x = 1; x < GRID_W - 1; x++) {
      int dvx = (int)vx[idx(x,y)] - ((int)p[idx(x+1,y)] - (int)p[idx(x-1,y)]) / 2;
      int dvy = (int)vy[idx(x,y)] - ((int)p[idx(x,y+1)] - (int)p[idx(x,y-1)]) / 2;
      vx[idx(x,y)] = (vel_t)(dvx > VEL_MAX ? VEL_MAX : (dvx < VEL_MIN ? VEL_MIN : dvx));
      vy[idx(x,y)] = (vel_t)(dvy > VEL_MAX ? VEL_MAX : (dvy < VEL_MIN ? VEL_MIN : dvy));
    }
  }
}

// Enforce boundary conditions:
// - Reflect normal velocity at walls so fluid bounces/splashes back
// - Density mirrors the interior cell (no-flux, fluid piles up at wall)
static void set_boundary(void) {
  // Left and right walls: reflect x-velocity, copy y-velocity and density
  for (int y = 0; y < GRID_H; y++) {
    // left wall: negate x so fluid bounces rightward
    s_vx[idx(0,        y)] = (vel_t)(-(int)s_vx[idx(1,        y)]);
    // right wall: negate x so fluid bounces leftward
    s_vx[idx(GRID_W-1, y)] = (vel_t)(-(int)s_vx[idx(GRID_W-2, y)]);
    s_vy[idx(0,        y)] =  s_vy[idx(1,        y)];
    s_vy[idx(GRID_W-1, y)] =  s_vy[idx(GRID_W-2, y)];
    s_density[idx(0,        y)] = s_density[idx(1,        y)];
    s_density[idx(GRID_W-1, y)] = s_density[idx(GRID_W-2, y)];
  }
  // Top and bottom walls: reflect y-velocity, copy x-velocity and density
  for (int x = 0; x < GRID_W; x++) {
    s_vy[idx(x, 0       )] = (vel_t)(-(int)s_vy[idx(x, 1       )]);
    s_vy[idx(x, GRID_H-1)] = (vel_t)(-(int)s_vy[idx(x, GRID_H-2)]);
    s_vx[idx(x, 0       )] =  s_vx[idx(x, 1       )];
    s_vx[idx(x, GRID_H-1)] =  s_vx[idx(x, GRID_H-2)];
    s_density[idx(x, 0       )] = s_density[idx(x, 1       )];
    s_density[idx(x, GRID_H-1)] = s_density[idx(x, GRID_H-2)];
  }
}

static void apply_noise(void) {
  for (int y = 1; y < GRID_H - 1; y++) {
    for (int x = 1; x < GRID_W - 1; x++) {
      if (s_density[idx(x, y)] < 8) continue;
      int nvx = (int)s_vx[idx(x,y)] + fast_rand(1);
      int nvy = (int)s_vy[idx(x,y)] + fast_rand(1);
      s_vx[idx(x,y)] = (vel_t)(nvx > VEL_MAX ? VEL_MAX : (nvx < VEL_MIN ? VEL_MIN : nvx));
      s_vy[idx(x,y)] = (vel_t)(nvy > VEL_MAX ? VEL_MAX : (nvy < VEL_MIN ? VEL_MIN : nvy));
    }
  }
}

// Splash: where the jets pile up against the left (back) wall, the fluid can't
// keep going left, so it sprays up and down along the wall. We make this happen
// by pushing each wall cell toward its emptier vertical neighbour — fluid flows
// off the top of the pile and off the bottom, fanning out into a splash. This is
// emergent: it works wherever the stream lands and for one jet or both.
static void splash_left_wall(void) {
  // Only the few columns hugging the left wall participate in the splash.
  for (int x = 1; x <= 3; x++) {
    for (int y = 1; y < GRID_H - 1; y++) {
      if (s_density[idx(x, y)] < 32) continue;  // no fluid here to splash

      // Spread from high density to low: if there's more fluid below than above,
      // push this cell up (toward the emptier side), and vice-versa.
      int grad = (int)s_density[idx(x, y+1)] - (int)s_density[idx(x, y-1)];
      // Stronger nearer the wall (x==1), tapering outward.
      int push = -grad * (4 - x) / 12;
      int nvy = (int)s_vy[idx(x, y)] + push;
      s_vy[idx(x, y)] = (vel_t)clamp_i(nvy, VEL_MIN, VEL_MAX);
    }
  }
}

// Viscosity: minimal smoothing — just enough to fill frame-to-frame gaps.
// a = 1/32: (32*self + sum) / 36
static void diffuse_vel(vel_t *v) {
  static vel_t tmp[N_CELLS_MAX];
  memcpy(tmp, v, sizeof(tmp));
  for (int y = 1; y < GRID_H - 1; y++) {
    for (int x = 1; x < GRID_W - 1; x++) {
      int sum = (int)tmp[idx(x-1,y)] + (int)tmp[idx(x+1,y)]
              + (int)tmp[idx(x,y-1)] + (int)tmp[idx(x,y+1)];
      int nv = (32 * (int)tmp[idx(x,y)] + sum) / 36;
      v[idx(x,y)] = (vel_t)(nv > VEL_MAX ? VEL_MAX : (nv < VEL_MIN ? VEL_MIN : nv));
    }
  }
}

static void fluid_step(void) {
  // --- velocity step ---
  apply_noise();
  splash_left_wall();   // fan fluid up/down where it hits the back wall
  add_source_vel(s_vx, s_vx_prev);
  add_source_vel(s_vy, s_vy_prev);

  // viscosity — spread momentum so the jet stays continuous
  diffuse_vel(s_vx);
  diffuse_vel(s_vy);

  // swap and advect velocity
  memcpy(s_vx_prev, s_vx, sizeof(s_vx));
  memcpy(s_vy_prev, s_vy, sizeof(s_vy));
  advect_vel(s_vx, s_vx_prev, s_vx_prev, s_vy_prev);
  advect_vel(s_vy, s_vy_prev, s_vx_prev, s_vy_prev);
  project(s_vx, s_vy);

  // --- density step ---
  add_source_density(s_density, s_density_prev);

  // Skip diffusion (it loses mass); advection alone moves the dye.
  memcpy(s_density_prev, s_density, sizeof(s_density));
  advect_density(s_density, s_density_prev, s_vx, s_vy);

  // Gentle dissipation so fluid fades over time rather than accumulating forever
  for (int i = 0; i < N_CELLS; i++) {
    s_density[i] = (uint8_t)((s_density[i] * 255) >> 8);
  }

  set_boundary();

  // clear sources for next frame
  clear_prev();
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

#ifdef PBL_COLOR
// 4x4 Bayer matrix, values 0..15 used as dither thresholds
static const uint8_t BAYER4[4][4] = {
  {  0,  8,  2, 10 },
  { 12,  4, 14,  6 },
  {  3, 11,  1,  9 },
  { 15,  7, 13,  5 },
};

// Quantize one 0..255 channel to the Pebble's 4 levels (0,85,170,255)
// with ordered dithering using the supplied Bayer threshold (0..15).
static inline uint8_t dither_channel(int v, uint8_t bayer) {
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  int level = v / 85;            // 0..3 (floor)
  int rem   = v - level * 85;    // 0..84 remainder toward next level
  // bayer 0..15 -> threshold 0..84; if remainder exceeds it, bump up a level
  if (rem * 16 > (int)bayer * 85 && level < 3) level++;
  return (uint8_t)(level * 85);
}

// Map density 0-255 to a blue->cyan->white gradient with ordered dithering.
static GColor density_to_color(uint8_t d, int gx, int gy) {
  int r, g, b;
  if (d < 64) {
    int t = d * 3 / 64;            // 0..2
    r = 0; g = 0; b = t * 85;
  } else if (d < 128) {
    int t = (d - 64);
    r = 0; g = 0; b = 85 + t * 170 / 64;
  } else if (d < 192) {
    int t = (d - 128);
    r = 0; g = t * 255 / 64; b = 255;
  } else {
    int t = (d - 192);
    r = t * 255 / 63; g = 255; b = 255;
  }
  uint8_t bayer = BAYER4[gy & 3][gx & 3];
  return GColorFromRGB(dither_channel(r, bayer),
                       dither_channel(g, bayer),
                       dither_channel(b, bayer));
}
#endif

static void canvas_update_proc(Layer *layer, GContext *ctx) {
  GRect bounds = layer_get_bounds(layer);
  (void)bounds;

  // Clear background
  graphics_context_set_fill_color(ctx, GColorBlack);
  graphics_fill_rect(ctx, bounds, 0, GCornerNone);

  for (int y = 0; y < GRID_H; y++) {
    for (int x = 0; x < GRID_W; x++) {
      uint8_t d = s_density[idx(x, y)];
      if (d < 2) continue; // skip near-zero cells

      GRect cell = GRect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE);

#ifdef PBL_COLOR
      graphics_context_set_fill_color(ctx, density_to_color(d, x, y));
#else
      // 1-bit: threshold dither
      if (d < 64)       graphics_context_set_fill_color(ctx, GColorBlack);
      else if (d < 128) graphics_context_set_fill_color(ctx, GColorDarkGray);
      else if (d < 192) graphics_context_set_fill_color(ctx, GColorLightGray);
      else              graphics_context_set_fill_color(ctx, GColorWhite);
#endif

      graphics_fill_rect(ctx, cell, 0, GCornerNone);
    }
  }
}

// ---------------------------------------------------------------------------
// Timer
// ---------------------------------------------------------------------------
static void add_fluid_top(void);
static void add_fluid_bottom(void);
static void stir(void);

static void timer_callback(void *context) {
  (void)context;
  // Inject every frame while button is held — no repeat delay
  if (s_btn_up)     add_fluid_top();
  if (s_btn_down)   add_fluid_bottom();
  if (s_btn_select) stir();
  fluid_step();
  layer_mark_dirty(s_canvas_layer);
  s_timer = app_timer_register(FRAME_MS, timer_callback, NULL);
}

// ---------------------------------------------------------------------------
// Button handlers
// ---------------------------------------------------------------------------

// Density ceiling at the nozzle (below 255 to preserve gradient range —
// bright white appears where jets collide and concentrate)
#define INJECT_DENSITY 150

// Top jet: angled inward (downward) so it meets the bottom jet near center
static void add_fluid_top(void) {
  int cy = GRID_H / 5;
  for (int y = cy - 2; y <= cy + 2; y++) {
    if (y < 1 || y >= GRID_H - 1) continue;
    int dist = (y > cy) ? (y - cy) : (cy - y);  // 0..2
    int ceil = INJECT_DENSITY - dist * 20;       // taper edges
    for (int x = GRID_W - 4; x <= GRID_W - 2; x++) {
      if ((int)s_density[idx(x, y)] < ceil)
        s_density[idx(x, y)] = (uint8_t)ceil;
      s_vx[idx(x, y)] = (vel_t)clamp_i(-96 + fast_rand(4), VEL_MIN, VEL_MAX);
      s_vy[idx(x, y)] = (vel_t)clamp_i(57 + fast_rand(4), VEL_MIN, VEL_MAX);
    }
  }
}

// Bottom jet: angled inward (upward) so it meets the top jet near center
static void add_fluid_bottom(void) {
  int cy = GRID_H - (GRID_H / 5);
  for (int y = cy - 2; y <= cy + 2; y++) {
    if (y < 1 || y >= GRID_H - 1) continue;
    int dist = (y > cy) ? (y - cy) : (cy - y);
    int ceil = INJECT_DENSITY - dist * 20;
    for (int x = GRID_W - 4; x <= GRID_W - 2; x++) {
      if ((int)s_density[idx(x, y)] < ceil)
        s_density[idx(x, y)] = (uint8_t)ceil;
      s_vx[idx(x, y)] = (vel_t)clamp_i(-96 + fast_rand(4), VEL_MIN, VEL_MAX);
      s_vy[idx(x, y)] = (vel_t)clamp_i(-57 + fast_rand(4), VEL_MIN, VEL_MAX);
    }
  }
}

// Stir: inject rotational velocity directly into the live field
static void stir(void) {
  int cx = GRID_W / 2;
  int cy = GRID_H / 2;
  int radius = GRID_W / 3;
  for (int y = cy - radius; y <= cy + radius; y++) {
    for (int x = cx - radius; x <= cx + radius; x++) {
      if (x < 1 || x >= GRID_W - 1) continue;
      if (y < 1 || y >= GRID_H - 1) continue;
      int dx = x - cx;
      int dy = y - cy;
      int nvx = (int)s_vx[idx(x,y)] + (-dy * VEL_SCALE / 4);
      int nvy = (int)s_vy[idx(x,y)] + ( dx * VEL_SCALE / 4);
      s_vx[idx(x,y)] = (vel_t)(nvx > VEL_MAX ? VEL_MAX : (nvx < VEL_MIN ? VEL_MIN : nvx));
      s_vy[idx(x,y)] = (vel_t)(nvy > VEL_MAX ? VEL_MAX : (nvy < VEL_MIN ? VEL_MIN : nvy));
    }
  }
}

// Raw press/release — inject immediately on press for instant response,
// then the timer loop continues injecting each frame while held.
static void up_press(ClickRecognizerRef r, void *ctx) {
  (void)r; (void)ctx;
  s_btn_up = true;
  add_fluid_top();
}
static void up_release(ClickRecognizerRef r, void *ctx)   { (void)r;(void)ctx; s_btn_up = false; }

static void down_press(ClickRecognizerRef r, void *ctx) {
  (void)r; (void)ctx;
  s_btn_down = true;
  add_fluid_bottom();
}
static void down_release(ClickRecognizerRef r, void *ctx) { (void)r;(void)ctx; s_btn_down = false; }

static void select_press(ClickRecognizerRef r, void *ctx) {
  (void)r; (void)ctx;
  s_btn_select = true;
  stir();
}
static void select_release(ClickRecognizerRef r, void *ctx) { (void)r;(void)ctx; s_btn_select = false; }

static void click_config_provider(void *context) {
  (void)context;
  window_raw_click_subscribe(BUTTON_ID_UP,     up_press,     up_release,     NULL);
  window_raw_click_subscribe(BUTTON_ID_DOWN,   down_press,   down_release,   NULL);
  window_raw_click_subscribe(BUTTON_ID_SELECT, select_press, select_release, NULL);
}

// ---------------------------------------------------------------------------
// Window lifecycle
// ---------------------------------------------------------------------------
static void window_load(Window *window) {
  Layer *window_layer = window_get_root_layer(window);
  GRect bounds = layer_get_bounds(window_layer);

  s_canvas_layer = layer_create(bounds);
  layer_set_update_proc(s_canvas_layer, canvas_update_proc);
  layer_add_child(window_layer, s_canvas_layer);

  window_set_click_config_provider(window, click_config_provider);

  // Initialize simulation state
  memset(s_density,      0, sizeof(s_density));
  memset(s_density_prev, 0, sizeof(s_density_prev));
  memset(s_vx,           0, sizeof(s_vx));
  memset(s_vy,           0, sizeof(s_vy));
  memset(s_vx_prev,      0, sizeof(s_vx_prev));
  memset(s_vy_prev,      0, sizeof(s_vy_prev));

  // Start sim timer
  s_timer = app_timer_register(FRAME_MS, timer_callback, NULL);
}

static void window_unload(Window *window) {
  (void)window;
  if (s_timer) {
    app_timer_cancel(s_timer);
    s_timer = NULL;
  }
  layer_destroy(s_canvas_layer);
  s_canvas_layer = NULL;
}

// ---------------------------------------------------------------------------
// App lifecycle
// ---------------------------------------------------------------------------
static void init(void) {
  s_window = window_create();
  window_set_background_color(s_window, GColorBlack);
  window_set_window_handlers(s_window, (WindowHandlers) {
    .load   = window_load,
    .unload = window_unload,
  });
  window_stack_push(s_window, true);
}

static void deinit(void) {
  window_destroy(s_window);
}

int main(void) {
  init();
  app_event_loop();
  deinit();
}
