
# Lista de trabajo propuesto a futuro.


### Representación episódica

- Where: Crear submensaje para Where.msg, que almacene: location, area, time
- Where: Soportar más de 1 lugar semántico y coordenado para hijos.
- Where: agregar soporte a más de un frame/mapa
- Where: Crear submensaje para Where.msg, que represente ubicación coordenada múltiple
- Optimización: condición de que hijos son ordenados por tiempo de inicio
- Asignación creciente de IDS para episodios, streams y entidades.

### Funcionalidad

- Modificación de la estructura de mensajes semánticos
- como manejar contextos, pues se puede requerir su información, incluso durante el mismo episodio!!.. deben ser almacenados una vez inscritos y ser cerrados manualmente.


### API ROS

- Quitar servicios incompletos.. 


### API plugins

### Herramientas

- Server con warning si mensaje a almacenar pesa más del límite!.
- LTM server con warning en texto y notificación inotify cuando quede poco espacio de disco (menor a X%) (o se ha ocupado más de X GB) debido al server... configurable por usuario...