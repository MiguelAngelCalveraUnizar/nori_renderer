#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
NORI_NAMESPACE_BEGIN
class PathTracing : public Integrator
{
public:
    PathTracing(const PropertyList &props)
    {
        /* No parameters this time */
    }
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f &ray) const
    {
        Color3f Lo(0.); // Total radiance
        Color3f Le(0.); // Emitter radiance
        Color3f fr(1); // Accumulation of f*cos/p
        bool keepTracing = true;
        Ray3f next_ray(ray);
        Intersection its;
        float rr_limit = 0.9f;
        size_t n_bounces = 0;
        while (keepTracing && fr.mean() > 0) { // If it won't give light stop
            // 1:
            // Find the surface that is visible in the requested direction
            if (scene->rayIntersect(next_ray, its)) {
                // If it intersect but it's not an emitter create another bounce with some prob.
                if (!its.mesh->isEmitter()) {
                    // If it's not an emitter, keep sampling
                    //keepTracing = true; (not needed)
                    // 2 Sample with BSDF:
                    BSDFQueryRecord bsdfRecord(its.toLocal(-next_ray.d), its.uv);
                    // We sample a direction wo with probability proportional to the BSDF. Then we get the fr*cos/p_omega            
                    fr *= its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());
                    // New intersection and new ray:
                    next_ray = Ray3f(its.p, its.toWorld(bsdfRecord.wo));
                    // For RR saying to not continuing with probability of sample()
                    rr_limit = std::min(0.9f, (std::max(fr[0], std::max(fr[1], fr[2]))));
                }
                else {
                    // If it does intersect with an emitter stop the bouncing and add up the contribution
                    keepTracing = false;
                    EmitterQueryRecord emitterRecord(its.mesh->getEmitter(), next_ray.o, its.p, its.shFrame.n, its.uv);
                    Le = its.mesh->getEmitter()->eval(emitterRecord);
                }
            }
            else {
                // If it doesn't intersect stop, and get Background (enviromental light as well)
                keepTracing = false;
                Le = scene->getBackground(next_ray);
            }

            // Extra end conditions:
            // not continuing with probability of sample() accumulated
            float rnd = (float)rand() / RAND_MAX;
            keepTracing = keepTracing && (rnd < rr_limit || n_bounces < 2); //90% of continuing.
            n_bounces+=1;
        }
        Lo = fr * Le;
        
        return Lo;
    }
    std::string toString() const
    {
        return "Direct Whitted Integrator []";
    }
};
NORI_REGISTER_CLASS(PathTracing,"path");
NORI_NAMESPACE_END