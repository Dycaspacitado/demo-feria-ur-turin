import './styles.css';
import { App } from './ui/app';

const app = new App(document.getElementById('app')!);
// acceso de depuración solo en modo desarrollo (npm run dev)
if (import.meta.env.DEV) (window as unknown as { __app: App }).__app = app;
