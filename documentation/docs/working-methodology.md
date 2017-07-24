# Metodología de trabajo

*Para desarrolladores.*

## Manejo del Repositorio

### Fork + Pull requests

Usamos el repositorio firmware_v2 del repositorio de github del proyecto CIAA
(este repo), mediante el mecanismo de hacer un clone a nuestas cuentas
personales de github y manejarnos con *pull requests*.

**Nota:** *No es algo de github sino que es un concepto propio de git, solo que github lo arma con una interfaz grafica "amigable" pero básicamente, es mandar parches con un formato especial. Ver: <https://linux.die.net/man/1/git-request-pull>*

### Issues

Se va a abrir un *issue* por cada cosa a mejorar.

## Producción de código y su documentación

Escribir los *.h con todas las cosas, documentarlos con doxygen y luego recién
ahi, empezar a escribir la implementación (supongo que todo va a ir primero a
la edu-ciaa-nxp).

**Nota:** *El problema de documentar separado del codigo, es lo de siempre, se desincroniza. La documentació ni tiene que estar separada del codigo, ni junto con el codigo, sino que ES EL CODIGO. De esta manera evitamos la desincronización.*

Luego como HTML y cualquier cosa que saque doxygen es complicada se utilizará
alguna herramienta para pasarlo a *markdown*. 

### Code Style

Usar profile de Eclipse provisto "CIAA-Eclipse-C-Profile.xml" (está en la
carpeta de ejemplos de la sAPI) con:

- 3 espacios en lugar de tabs (como el resto del firmware
- Sin espacios al final de la línea.

Además considerar:

- Limitar el ancho de texto a 80 caracteres. 
- No usar guión bajo en nombres de funciones, usar camelCase (como en la sAPI actual).
- Para variables también camelCase.
- Para constantes o macros MAYÚSCULAS_Y_GUIONES_BAJOS. (ver si permitimos algunas que parezcan funciones).
- para cualquier tipo de datos que definamos agregarle el sufijo "_t" (por ejemplo, GpioConfig_t, uint8_t, etc.).

- Usar bloques de código con llaves que abren en línea del if o función.
```c
if( saraza ) {
   statement1;
} else {
   statement2;
}
```

- Funciones muy largas en su uso o declaración hay que poner los parámetros en columna, ejemplo:

```c
retType funcionLaaaaaaaaaaaaaaaarrrrrrrgaaaaaaaaaaa( tipoDeDato_t param1,
                                                     tipoDeDato_t param2,
                                                     ...                  ) {
   statements;
}
```

O bien:

```c
retType funcionRecontraLaaaaaaaaaaaaaaaarrrrrrrgaaaaaaaaaaaaaaaaaaaaaaaa( 
   tipoDeDato_t param1,
   tipoDeDato_t param2,
   ...
) { 
   statements;
}
```

## Documentación general

Se utiliza Markdown ya que es muy sencillo y pudede verse directo en github o
gitlab. También se puede generar una vista web en la PC local con porgramas
como *mkdocs* y generar pdf.

Esta documentación será en parte obtenida desde el código convirtiendo
*doxigen* a *mkdocs*, ver:

- <https://www.npmjs.com/package/doxygen2md>
- <https://github.com/vstakhov/doxydown>

Además habrá otra parte de la documentación escrita como el presente texto que
está leyendo.

