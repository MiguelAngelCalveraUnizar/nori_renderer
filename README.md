# nori_renderer
Nori based renderer as part of MSoA course

This version includes part of the project to do a scarf as a heterogeneus anisotropic participating media

Lista de cosas por hacer:
- Hacer un Volumetric Path Integrator. La base será single_scat, pero tendra un efecto recursivo y Russian Roulette para terminar de ejecutarlo
- Añadir un archivo heterogeneus.cpp similar a homogeneus.cpp
- Añadir un archivo que se encargue de leer un .vol que contiene densidad de particulas y orientacion

Una vez añadido soporte para cargar archivos .vol con densidad de particulas y orientación
- Funcion de sampleo (como sampleBetween), que tiene que escoger el punto segun la densidad de particulas. (heterogeneus.cpp)
    - Sampleo tiene que ser mediante delta tracking, woodcok o null collision (los 3 son lo mismo).
    - Implementarlo
- Determinar Phase Funcion (similar a pf_fog.cpp, pero con microflake.cpp)
    - Investigar cual es la de microflake
    - Creo que la phase Function del microflake se determina mediante D(w) que es sin20(v, ω), pero no se como va.
    - Implementar
- Determinar Transmittance a partir de datos de .vol (heterogeneus.cpp) (Woodcock otra vez)
- Determinar Absortion coeficient a partir de datos de .vol (heterogeneus.cpp)
- Determinar Scattering coeficient a partir de datos de .vol (heterogeneus.cpp)
- Determinar Transmittance coefficient a partir de datos de .vol (heterogeneus.cpp)
    - Segun el paper utilizan "ray marching" para obtener simga_t: Pag 10 [A radiative transfer framework for rendering materials with anisotropic structure] 
    - Pero luego parece que acabaron usando delta tracking, woodcok o null collision (los 3 son lo mismo).

Puede que sea un problema más tarde:
- rayIntersectMedium supone que solo va a intersectar 1 vez con el medio para obtener los x y xz. Pero puede haber en el mismo mesh varias entradas y salidas del medio. O podemos hacer que se tapen esas entradas y salidas con una taza, como hemos hecho

DONE:
- Añadir una bounding box, o un mesh que sirva de limite para los efectos de un medio
    - Leer un mesh o un obj y cargarlo en memoria como un objeto que sirva para limitar al medio a esa geometria
    - Esta bounding box tiene que permitir para una direccion tener los 2 puntos de entrada y salida de un medio, o avisar de si no hay medio con algun metodo (rayIntersectionMedium se encarga de meter en el mediumIntersection que le pases los puntos de entrada y salida)
- Single Scattering añadir comprobaciones NaN en todas las divisiones por probabilidades o todas las operaciones 
- Error al hacer rayIntersectMedium. Ignora que haya otros objetos entre el punto o (camara) y el medio. Así que aporta transmittance a pesar de no haber objeto.
