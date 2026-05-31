// Pebble Fluid Sim - PebbleKit JS
// Shows the settings page and forwards the chosen color palette to the watch.

var PALETTES = [
  { id: 0, name: 'Water',   desc: 'Cool blue to cyan to white',
    css: 'linear-gradient(90deg,#001428,#0040ff,#00ffff,#ffffff)' },
  { id: 1, name: 'Fire',    desc: 'Red to orange to yellow to white',
    css: 'linear-gradient(90deg,#200000,#ff0000,#ffa500,#ffff00,#ffffff)' },
  { id: 2, name: 'Acid',    desc: 'Deep green to chartreuse to white',
    css: 'linear-gradient(90deg,#003c00,#00b400,#b4ff00,#ffffff)' },
  { id: 3, name: 'Rainbow', desc: 'Full spectrum cycling with density',
    css: 'linear-gradient(90deg,#ff0000,#ffa500,#ffff00,#00ff00,#00ffff,#0000ff,#ff00ff)' }
];

function buildConfigPage(selected) {
  var rows = '';
  for (var i = 0; i < PALETTES.length; i++) {
    var p = PALETTES[i];
    var checked = (p.id === selected) ? ' checked' : '';
    rows +=
      '<label class="opt">' +
        '<input type="radio" name="pal" value="' + p.id + '"' + checked + '>' +
        '<div class="body">' +
          '<div class="name">' + p.name + '</div>' +
          '<div class="desc">' + p.desc + '</div>' +
          '<div class="swatch" style="background:' + p.css + '"></div>' +
        '</div>' +
      '</label>';
  }

  return '<!DOCTYPE html><html><head>' +
    '<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">' +
    '<meta charset="utf-8">' +
    '<style>' +
      '*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}' +
      'body{margin:0;font-family:-apple-system,Roboto,Helvetica,Arial,sans-serif;' +
        'background:#111418;color:#fff;padding:18px}' +
      'h1{font-size:20px;font-weight:600;margin:4px 0 2px}' +
      'p.sub{color:#8a93a0;font-size:13px;margin:0 0 18px}' +
      '.opt{display:flex;align-items:center;gap:12px;background:#1b1f26;' +
        'border:2px solid #1b1f26;border-radius:14px;padding:14px;margin-bottom:12px;cursor:pointer}' +
      '.opt input{width:22px;height:22px;accent-color:#3b82f6;flex:0 0 auto}' +
      '.opt:has(input:checked){border-color:#3b82f6;background:#1f2733}' +
      '.body{flex:1 1 auto;min-width:0}' +
      '.name{font-size:16px;font-weight:600}' +
      '.desc{font-size:12px;color:#8a93a0;margin:2px 0 8px}' +
      '.swatch{height:16px;border-radius:8px;border:1px solid rgba(255,255,255,.15)}' +
      'button{position:sticky;bottom:14px;width:100%;border:0;border-radius:14px;' +
        'background:#3b82f6;color:#fff;font-size:17px;font-weight:600;padding:15px;margin-top:6px}' +
      'button:active{background:#2f6fd6}' +
    '</style></head><body>' +
      '<h1>Color palette</h1>' +
      '<p class="sub">Pick the gradient for the fluid.</p>' +
      '<form id="f">' + rows + '</form>' +
      '<button id="save">Save</button>' +
    '<script>' +
      'document.getElementById("save").addEventListener("click",function(){' +
        'var sel=document.querySelector("input[name=pal]:checked");' +
        'var pal=sel?parseInt(sel.value,10):0;' +
        'var out={palette:pal};' +
        'document.location="pebblejs://close#"+encodeURIComponent(JSON.stringify(out));' +
      '});' +
    '</script></body></html>';
}

Pebble.addEventListener('ready', function() {
  console.log('Fluid Sim PebbleKit JS ready');
});

Pebble.addEventListener('showConfiguration', function() {
  var saved = parseInt(localStorage.getItem('palette'), 10);
  if (isNaN(saved)) { saved = 0; }
  Pebble.openURL('data:text/html,' + encodeURIComponent(buildConfigPage(saved)));
});

Pebble.addEventListener('webviewclosed', function(e) {
  if (!e || !e.response) { return; }  // user backed out without saving
  var raw = e.response;
  var data;
  try {
    data = JSON.parse(decodeURIComponent(raw));
  } catch (err) {
    try { data = JSON.parse(raw); } catch (err2) { return; }
  }
  var pal = parseInt(data.palette, 10);
  if (isNaN(pal)) { return; }
  localStorage.setItem('palette', String(pal));
  Pebble.sendAppMessage({ PALETTE: pal },
    function() { console.log('palette sent: ' + pal); },
    function(err) { console.log('palette send failed'); });
});
