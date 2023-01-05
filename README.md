# nori_renderer
Nori based renderer as part of MSoA course

This version includes part of the project to do a scarf as a heterogeneus anisotropic participating media

Lista de cosas por hacer:
- Hacer un Volumetric Path Integrator. La base será single_scat, pero tendra un efecto recursivo y Russian Roulette para terminar de ejecutarlo
- Añadir un archivo heterogeneus.cpp similar a homogeneus.cpp
- Añadir un archivo que se encargue de leer un .vol que contiene densidad de particulas y orientacion
- Añadir una bounding box, o un mesh que sirva de limite para los efectos de un medio
    - Leer un mesh o un obj y cargarlo en memoria como un objeto que sirva para limitar al medio a esa geometria
    - Esta bounding box tiene que permitir para una direccion tener los 2 puntos de entrada y salida de un medio, o avisar de si no hay medio con algun metodo
    - Dicho metodo se utilizara en el samplingBetween de homogeneus.cpp para utilizarlo en vez de x y xz

Una vez añadido soporte para cargar archivos .vol con densidad de particulas y orientación
- Funcion de sampleo (como sampleBetween), que tiene que escoger el punto segun la densidad de particulas. (heterogeneus.cpp)
    - Investigar que tipo de sampleo hace falta
    - Implementarlo
- Determinar Phase Funcion (similar a pf_fog.cpp, pero con microflake.cpp)
    - Investigar cual es la de microflake
    - Implementar
- Determinar Transmittance a partir de datos de .vol (heterogeneus.cpp)
- Determinar Absortion coeficient a partir de datos de .vol (heterogeneus.cpp)
- Determinar Scattering coeficient a partir de datos de .vol (heterogeneus.cpp)


