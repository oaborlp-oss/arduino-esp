# Cómo contribuir

Reglas para que cada programador deje sus cambios ordenados y trazables.

## 1. Trabaja en una rama, no directo en `main`

```bash
git checkout main
git pull
git checkout -b tipo/descripcion-corta
```

Ejemplos de nombres: `feat/grafico-oled`, `fix/wifi-reconexion`, `docs/conexiones-s3`.

## 2. Mensajes de commit

Formato: `tipo(equipo): descripción en minúsculas`

| Tipo | Cuándo |
|---|---|
| `feat` | Función nueva |
| `fix` | Corrección de un error |
| `docs` | Solo documentación |
| `refactor` | Reorganizar código sin cambiar lo que hace |
| `test` | Sketches de prueba |
| `chore` | Mantenimiento (configuración, archivos auxiliares) |

Equipos: `monitor`, `cam-oled`, `pruebas`, `docs`.

Ejemplos:
- `feat(cam-oled): agrega mínimos y máximos de 24 h`
- `fix(monitor): corrige reconexión WiFi tras corte de luz`

## 3. Actualiza el CHANGELOG en cada cambio

En [CHANGELOG.md](CHANGELOG.md), agrega una línea bajo **`## [Sin publicar]`**, en la
sección que corresponda:

| Sección | Para |
|---|---|
| `### Agregado` | Funciones nuevas |
| `### Cambiado` | Cambios en funciones existentes |
| `### Corregido` | Errores arreglados |
| `### Eliminado` | Funciones quitadas |
| `### Seguridad` | Vulnerabilidades o datos sensibles |

Formato de la línea:

```markdown
- **[Equipo]** Qué cambió, explicado para quien usa el equipo. — @tu-usuario
```

Si el cambio obliga a **recablear**, dilo explícitamente en la línea (pines antes → después).

## 4. Antes de abrir el Pull Request

- [ ] Compila sin errores en el Arduino IDE para cada placa afectada.
- [ ] Si se pudo, se probó en la placa real (indica cuál).
- [ ] `secrets.h` **no** está incluido (nunca subir claves WiFi).
- [ ] CHANGELOG actualizado.

GitHub revisa automáticamente que el Pull Request modifique `CHANGELOG.md`. Si el cambio
de verdad no lo necesita (por ejemplo, un error de tipeo en un comentario), agrega la
etiqueta **`sin-changelog`** al Pull Request.

## 5. Publicar una versión (responsable del proyecto)

1. En CHANGELOG.md, cambia `## [Sin publicar]` por `## [X.Y.Z] - AAAA-MM-DD` y crea un
   `## [Sin publicar]` vacío arriba.
2. Actualiza los enlaces de comparación al final del archivo.
3. Haz commit: `chore: versión X.Y.Z`.
4. Crea la etiqueta y súbela:

```bash
git tag -a vX.Y.Z -m "Versión X.Y.Z"
git push origin main --tags
```
