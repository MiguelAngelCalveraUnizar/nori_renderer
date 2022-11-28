#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
NORI_NAMESPACE_BEGIN
class PathTracingNEE : public Integrator
{
public:
    PathTracingNEE(const PropertyList &props)
    {
        /* No parameters this time */
    }
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Color3f Lo(0.); // Total radiance
        Color3f Le(0.); // Emitter radiance
        Color3f fr(1); // Accumulation of f*cos/p
        // Last bsdf measure. To know if we should take into account light sources that were hit using BSDF sampling
        EMeasure measure_last_bsdf = EDiscrete; // Starts as EDiscrete, because if the first ray (from camera) is a light we do have to take it into account
        bool keepTracing = true;
        Ray3f next_ray(ray);
        Intersection its;
        float rr_limit = 0.9f;
        size_t n_bounces = 0;
        
        while (keepTracing && fr.getLuminance() > 0) { // If it won't give light stop 
            // 1:
            Color3f Li(0.); // Incoming radiance
            // Find the surface that is visible in the requested direction
            if (scene->rayIntersect(next_ray, its)) {
                // If it intersect but it's not an emitter create another bounce with some prob.
                if (!its.mesh->isEmitter()) {
                    // If it's not an emitter, add contribution from a sampled light and keep sampling
                    //keepTracing = true; (not needed)
                    // __________________________________________________________________
                    // 2 Sample emitter
                    //Sample randomly a light source
                    float pdf_light;
                    const Emitter* light = scene->sampleEmitter(sampler->next1D(), pdf_light);
                    //Sample the light
                    EmitterQueryRecord emitterRecord(its.p);
                    Li = light->sample(emitterRecord, sampler->next2D(), 0.);

                    // 3 Check visibility of emitter
                    float V = 1;
                    Ray3f sray(its.p, emitterRecord.wi);
                    Intersection it_shadow;
                    if (scene->rayIntersect(sray, it_shadow)) {
                        if (it_shadow.t < (emitterRecord.dist - 1.e-5)) {
                            V = 0;
                        }
                    }
                    //BSDF 
                    BSDFQueryRecord EmitterBsdfRecord(its.toLocal(-next_ray.d),
                        its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);

                    //Probability of the sample of the point of the light source
                    float pdf_light_point = light->pdf(emitterRecord);
                    Color3f fr_light = (its.mesh->getBSDF()->eval(EmitterBsdfRecord) * its.shFrame.n.dot(emitterRecord.wi)) / (pdf_light * pdf_light_point);
                    
                    // 4 Add up contribution of emitter
                    Lo += V * Li * fr * fr_light;

                    // __________________________________________________________________
                    // 5 Sample with BSDF:
                    BSDFQueryRecord bsdfRecord(its.toLocal(-next_ray.d), its.uv);
                    // We sample a direction wo with probability proportional to the BSDF. Then we get the fr*cos/p_omega            
                    fr *= its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());
                    measure_last_bsdf = bsdfRecord.measure;
                    // New intersection and new ray:
                    next_ray = Ray3f(its.p, its.toWorld(bsdfRecord.wo));
                    // For RR saying to not continuing with probability of sample()
                    rr_limit = std::min(0.9f, (std::max(fr[0], std::max(fr[1], fr[2]))));
                }
                else {
                    // If it does intersect with an emitter stop the bouncing and add up the contribution
                    // if the last measure is EDiscrete, we have to take into account the light intersected with BSDF, 
                    // if not, then we shouldn't take it into account.
                    keepTracing = false;
                    if (measure_last_bsdf == EDiscrete) {
                        //cout << "Nay";
                        EmitterQueryRecord emitterRecord(its.mesh->getEmitter(), next_ray.o, its.p, its.shFrame.n, its.uv);
                        Le = its.mesh->getEmitter()->eval(emitterRecord);
                        Lo += fr * Le;
                    }
                }
            }
            else {
                // If it doesn't intersect stop, and get Background (enviromental light as well)
                keepTracing = false;
                // if the last measure is EDiscrete, we have to take into account the light intersected with BSDF, 
                // if not, then we shouldn't take it into account.
                if (measure_last_bsdf == EDiscrete) {
                    Le = scene->getBackground(next_ray);
                    Lo += fr * Le;
                }
            }

            // Extra end conditions:
            // not continuing with probability of sample() accumulated
            float rnd = (float)rand() / RAND_MAX;
            keepTracing = keepTracing && (rnd < rr_limit || n_bounces < 3); //90% of continuing.
            n_bounces += 1;
        }
        return Lo;
    }
    std::string toString() const
    {
        return "Direct Whitted Integrator []";
    }
};
NORI_REGISTER_CLASS(PathTracingNEE,"path_nee");
NORI_NAMESPACE_END