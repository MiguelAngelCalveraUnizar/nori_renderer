#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
NORI_NAMESPACE_BEGIN
class DirectMaterialSampling : public Integrator
{
public:
    DirectMaterialSampling(const PropertyList &props)
    {
        /* No parameters this time */
    }
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f &ray) const
    {
        Color3f Lo(0.); // Total radiance
        Color3f Le(0.); // From the first intersection (If its emitter only)
        Color3f Li(0.); // From the bsdf generated-intersection (If next_intersection is emitter)
        // Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return scene->getBackground(ray);
        }
        
        BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
        // Wo and wi estan cambiados.

        // We sample a direction wo with probability proportional to the BSDF. Then we get the fr*cos/p_omega            
        Color3f fr = its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());

        //if (isnan(fr.mean()) || isnan(fr[0]) || isnan(fr[1]) || isnan(fr[2])) {
        //    cout << "Ups" << endl;
        //}
        bsdfRecord.measure = ESolidAngle;
        // Ray from p with wo to see if it intersects in a emitter
        Ray3f next_ray(its.p, bsdfRecord.wo);
        Intersection it_next;
        if (scene->rayIntersect(next_ray, it_next)){
            // If it intersects with something, then we check if the intersection is in a emitter.
            if (it_next.mesh->isEmitter()) {
                Li = it_next.mesh->getEmitter()->eval(EmitterQueryRecord(it_next.p));
                Li = Li * fr;
            }
        }
        else {
            // If it doesn't intersect, then we get the value from the background. 
            // Especially important for enviroment lights
            Li = fr * scene->getBackground(next_ray); // For some reason getBackground sometimes gives Nan
            if (isnan(Li.mean())) {
                Li = Color3f(0.0f);
            }
        }
        float Li_mean_check = Li.mean();
        if (isnan(Li_mean_check)) {
            cout << "what" << endl;
        }
        // If we intersected at first with an Emitter add the Le from it.
        if (its.mesh->isEmitter()) {
            Le = its.mesh->getEmitter()->eval(EmitterQueryRecord(ray.o));
        }

        // Sum all
        Lo = Le + Li;
        return Lo;
    }
    std::string toString() const
    {
        return "Direct Whitted Integrator []";
    }
};
NORI_REGISTER_CLASS(DirectMaterialSampling,"direct_mats");
NORI_NAMESPACE_END