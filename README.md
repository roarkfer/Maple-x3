# Maple X3 Receiver — v0.1 GitHub build

Primer firmware de diagnóstico para convertir el XTEINK X3 en receptor de Maple.

## Qué hace esta versión

Todavía **no sincroniza datos**. Su único objetivo es validar:

1. Que GitHub Actions puede compilar un firmware para el X3.
2. Que FreeInk inicializa correctamente tu pantalla.
3. Que el firmware detecta la variante de controlador de pantalla del lote.
4. Que la orientación del framebuffer es correcta.

Al flashearlo debe mostrar:

- `MAPLE X3`
- `GITHUB BUILD`
- `READY`

## Compilar sin instalar nada

1. Sube estos archivos a un repositorio llamado `Maple-X3`.
2. Abre la pestaña **Actions**.
3. Entra a **Build Maple X3 firmware**.
4. Pulsa **Run workflow**.
5. Al terminar abre el job y descarga el artifact **Maple-X3-firmware**.
6. Dentro estará `firmware.bin`.

GitHub Actions clona FreeInk SDK y usa la misma familia de toolchain Pioarduino que
usa actualmente CrossPoint para sus builds.

## Flasheo

Esta versión es experimental. Conserva una copia de tu firmware actual antes de
flashear. Usa el método de flasheo que ya usas en tu X3 desbloqueado.

## Siguiente versión

Cuando esta pantalla aparezca correctamente, v0.2 añadirá:

- Wi-Fi / hotspot `MAPLE-X3`
- endpoint HTTP para recibir `state.json`
- almacenamiento local
- pantalla de Tareas / Hábitos / Gym / Proyectos

Después podremos evaluar BLE para sincronización directa desde Maple.
