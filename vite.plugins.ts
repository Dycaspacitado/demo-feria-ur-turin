import type { Plugin } from 'vite';

/**
 * Convierte la compilación en UN solo archivo HTML autocontenido: inserta el JS y el CSS
 * dentro del index.html y elimina los archivos sueltos. Se activa con `vite build --mode single`.
 */
export function singleFileHtml(outputName: string, opts: { stripDocument?: boolean; title?: string } = {}): Plugin {
  return {
    name: 'single-file-html',
    apply: 'build',
    enforce: 'post',
    generateBundle(_options, bundle) {
      const html = bundle['index.html'];
      if (!html || html.type !== 'asset') return;
      let source = String(html.source);
      for (const [name, item] of Object.entries(bundle)) {
        const ref = new RegExp(`<script[^>]*src="[^"]*${escapeRe(name)}"[^>]*></script>`);
        if (item.type === 'chunk' && ref.test(source)) {
          // "</script" dentro del código cerraría la etiqueta: se escapa.
          const code = item.code.replace(/<\/script/gi, '<\\/script');
          source = source.replace(ref, () => `<script type="module">\n${code}\n</script>`);
          delete bundle[name];
        } else if (item.type === 'asset' && name.endsWith('.css')) {
          const link = new RegExp(`<link[^>]*href="[^"]*${escapeRe(name)}"[^>]*>`);
          source = source.replace(link, () => `<style>\n${String(item.source)}\n</style>`);
          delete bundle[name];
        }
      }
      delete bundle['index.html'];
      if (opts.title) source = source.replace(/<title>[^<]*<\/title>/, `<title>${opts.title}</title>`);
      if (opts.stripDocument) source = stripDocument(source);
      this.emitFile({ type: 'asset', fileName: outputName, source });
    },
  };
}

/**
 * Quita el envoltorio del documento (<!doctype>, <html>, <head>, <body>) para publicar como página
 * de Artifact, que añade su propio esqueleto. Orden: título, estilos, contenido del body, script.
 */
function stripDocument(html: string): string {
  const title = /<title>[\s\S]*?<\/title>/.exec(html)?.[0] ?? '';
  const styles = html.match(/<style>[\s\S]*?<\/style>/g) ?? [];
  const scripts = html.match(/<script type="module">[\s\S]*?<\/script>/g) ?? [];
  let body = /<body[^>]*>([\s\S]*)<\/body>/.exec(html)?.[1] ?? '';
  for (const s of scripts) body = body.replace(s, '');
  return [title, ...styles, body.trim(), ...scripts].join('\n');
}

const escapeRe = (s: string) => s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
