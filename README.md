# nori_renderer
Nori based renderer as part of MSoA course

This version includes part of the project to do a scarf as a heterogeneus anisotropic participating media

Lista de cosas por hacer:
- Corregir Volumetric Path integrator
- Hacer Normal Bumping


Puede que sea un problema más tarde:
- rayIntersectMedium supone que solo va a intersectar 1 vez con el medio para obtener los x y xz. Pero puede haber en el mismo mesh varias entradas y salidas del medio. O podemos hacer que se tapen esas entradas y salidas con una taza, como hemos hecho
