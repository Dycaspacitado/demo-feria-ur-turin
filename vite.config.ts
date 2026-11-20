import { defineConfig } from 'vite';
import { singleFileHtml } from './vite.plugins';

/**
 * Modos:
 *  - (por defecto)  aplicación normal en dist/
 *  - single         un único Simulacion-GRUPRO.html autocontenido (doble clic, sin servidor ni internet)
 */
export default defineConfig(({ mode }) => {
  const single = mode === 'single';
  return {
    base: './',
    plugins: single ? [singleFileHtml('Simulacion-GRUPRO.html')] : [],
    server: { port: 5180, open: false },
    build: single
      ? { outDir: 'dist-single', assetsInlineLimit: Number.MAX_SAFE_INTEGER, cssCodeSplit: false, chunkSizeWarningLimit: 4000, emptyOutDir: true }
      : { chunkSizeWarningLimit: 4000 },
  };
});
