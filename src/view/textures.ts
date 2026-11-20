import * as THREE from 'three';

/** Textura de texto dibujada en un canvas (letras de los cubos, logo del llavero, cartelería). */
export function textTexture(
  text: string,
  opts: { w?: number; h?: number; bg?: string; fg?: string; font?: string; size?: number; border?: string; sub?: string } = {},
): THREE.CanvasTexture {
  const w = opts.w ?? 256, h = opts.h ?? 256;
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  const g = c.getContext('2d')!;
  g.fillStyle = opts.bg ?? '#ffffff';
  g.fillRect(0, 0, w, h);
  if (opts.border) {
    g.strokeStyle = opts.border;
    g.lineWidth = Math.max(4, w * 0.025);
    g.strokeRect(g.lineWidth, g.lineWidth, w - 2 * g.lineWidth, h - 2 * g.lineWidth);
  }
  g.fillStyle = opts.fg ?? '#111';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.font = opts.font ?? `800 ${opts.size ?? Math.round(h * 0.72)}px "Segoe UI", Arial, sans-serif`;
  g.fillText(text, w / 2, opts.sub ? h * 0.42 : h / 2 + h * 0.03);
  if (opts.sub) {
    g.font = `600 ${Math.round(h * 0.14)}px "Segoe UI", Arial, sans-serif`;
    g.fillText(opts.sub, w / 2, h * 0.78);
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}

/** Cinta de seguridad amarilla y negra para demarcar la zona de los robots en el piso. */
export function hazardTexture(): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = 128; c.height = 32;
  const g = c.getContext('2d')!;
  g.fillStyle = '#f2c200';
  g.fillRect(0, 0, 128, 32);
  g.fillStyle = '#1b1b1b';
  for (let x = -32; x < 160; x += 32) {
    g.beginPath();
    g.moveTo(x, 32); g.lineTo(x + 16, 32); g.lineTo(x + 32, 0); g.lineTo(x + 16, 0);
    g.closePath(); g.fill();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

/** Piso de resina con una grilla tenue cada 500 mm. */
export function floorTexture(): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = c.height = 512;
  const g = c.getContext('2d')!;
  g.fillStyle = '#a9b0b7';
  g.fillRect(0, 0, 512, 512);
  g.strokeStyle = 'rgba(80,90,100,0.22)';
  g.lineWidth = 3;
  g.strokeRect(0, 0, 512, 512);
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}
