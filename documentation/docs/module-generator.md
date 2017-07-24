# Generador de módulos de la biblioteca

*Para desarrolladores.*

Este generador de módulos tiene que soportar se compone de las siguientes clases.



## Documentation

Documentación es un objeto que tiene un Dicccionario con asocieciones lenguaje->texto.
El texto indica el propósito de algo.

### Attributes

- name (String)

### Methods

- generate_doxygen_doc
- generate_markdown_doc




## DataType

Existen los siguientes tipos de datos:

- elementaryDataType (en c es int8_t, bool_t, etc)
- constantDataType (en c es un define o enum)
- moduleDataType (en c es una estructura, tipo un objeto)

### Attributes

- name (String)
- documentation (Documentation)

### Methods

- generate_c_code (List)
    - genetate_c_code()
        - generate_c_header_file()
        - generate_c_source_file()
- generate_java_code (List)
    - generate_java_code()
        - generate_java_class_file()
- generate_python_code (List)





## Attribute, Variable declaration, Formal parameter, Actual argument

Atributos (o propiedades) que tendrán cierto tipos de datos. Todos los atributos van a ser privados, accesibles mediante métodos getters y setters.

- name (String)
- documentation (Documentation)
- datatype (DataType)

### Attributes

### Methods




## Method

Definir métodos, con ciertos parámetros que tendrán ciertos tipos de datos y un cuerpo de método escrito en lenguaje C. Opcionalmente un método tendá además modificadores. Existirán 2 tipos de métodos:

- Métodos privados. Accesibles sólo por el módulo.
- Métodos públicos. Accesibles por el módulo y otros módulos.

### Attributes

- name (String)
- parameters (List)
- code (List)
    - c_code (String)
    - java_code (String)
    - python_code (String)
- returnType (DataType)
- scope (public, private)
- documentation (Documentation)

### Methods




## Module

Un módulo que contendrá: sus tipos de datos, dependencias de otros módulos, propiedades y métodos. Documentación que indica el propósito del módulo. Documentación que indica las restricciones de uso según board (tiene un conjunto de boards).
Pordá generar archivos en lenguaje C (.c y .h), con documentación en formato doxygen.

- modueleName.h, con headers y declaraciones públicas, independiente del hardware.
- modueleName.c, dependiente del hardware. Opcionalmente se podrá dividir en dos archivos, uno dependiente del hardware y uno independiente.

Puede generar automáticamente getters y setters de atributos.

### Attributes

- name (String)
- documentation (Documentation)
- datatype (DataType)
- instance_size (int)
- dependencies (List)
    - module (Module)
- attribute_categories (List)
    - configurarion (List)
        - power (Attribute)
        - clock_source (Attribute)
        - configurationParameterName (Attribute)
    - value (List)
        - valueName
    - polling_events (List)
       - eventName
    - interrupt_events (List)
        - interruptEventName
    - event_callbacks (List)
        - interruptEventNameCallback
- method_categories (List)
    - initialization (List)
        - initialize (Method)
    - attribute_getters (List)
    - attribute_setters (List)
    - configuration (List)
        - moduleConfig (Method)
    - read (List)
        - moduleRead (Method)
    - write (List)
        - moduleWrite (Method)

### Methods

- generate_c_code (List)
    - genetate_c_code()
        - generate_c_header_file()
        - generate_c_source_file()
- generate_java_code (List)
    - generate_java_code()
        - generate_java_class_file()
- generate_python_code (List)


