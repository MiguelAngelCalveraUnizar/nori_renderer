#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
NORI_NAMESPACE_BEGIN
class PathTracingMIS : public Integrator
{
public:
    PathTracingMIS(const PropertyList& props)
    {
        /* No parameters this time */
    }
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Color3f Lo(0.); // Total radiance
        Color3f Le(0.); // Emitter radiance
        Color3f fr(1); // Accumulation of f*cos/p
        bool keepTracing = true;
        Ray3f next_ray(ray);
        Intersection its;
        EMeasure measure_last_bsdf = EUnknownMeasure;
        // RR:
        float rr_limit = 0.9f;
        size_t n_bounces = 0;

        // MIS:
        float w_mat = 1; // We will add it in the next iteration and in the first it has to be 1.
        float w_em = 1;
        float p_em_wem = 1;
        float p_mat_wem = 1;
        float p_em_wmat = 1;
        float p_mat_wmat = 1;

        while (keepTracing && fr.getLuminance() > 0) { // If it won't give light stop 
            // 1:
            Le = Color3f(0.); // Emitter radiance
            // Find the surface that is visible in the requested direction
            if (scene->rayIntersect(next_ray, its)) {
                //Modify the normal shading if the bsdf has a normal map
                //Compute the new normal for bump mapping
                if (its.mesh->getBSDF()->hasDisplacementMap()) {
                    its.shading.n = its.shFrame.n + its.mesh->getBSDF()->displacement(its.uv);
                    its.shFrame = Frame(its.shading.n);
                }
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
                    Le = light->sample(emitterRecord, sampler->next2D(), 0.);

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
                    fr_light = fr_light.clamp();
                    // 3b) MIS get weigth w_em:
                    //Compute the p_em(sample ems) and p_mat(sample ems)
                    p_em_wem = pdf_light_point * pdf_light;
                    // For MIS we need to evaluate respect to p_mat_wem the pdf of the direction
                    p_mat_wem = its.mesh->getBSDF()->pdf(EmitterBsdfRecord);

                    w_em = (p_em_wem + p_mat_wem) > FLT_EPSILON ? p_em_wem / (p_em_wem + p_mat_wem) : 0;

                    // 4 Add up contribution of emitter
                    Lo += V * Le * fr * fr_light * w_em;

                    // __________________________________________________________________
                    // 5 Sample with BSDF:
                    BSDFQueryRecord bsdfRecord(its.toLocal(-next_ray.d), its.uv);
                    // We sample a direction wo with probability proportional to the BSDF. Then we get the fr*cos/p_omega            
                    fr *= its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());
                    measure_last_bsdf = bsdfRecord.measure;
                    // 5b) MIS get weight w_mats_last
                    // Get p_em_wmat
                    // p_em_wmat = it_next.mesh->getEmitter()->pdf(emitterRecordMat);
                    //Compute the p_mat(sample mats)
                    p_mat_wmat = its.mesh->getBSDF()->pdf(bsdfRecord);


                    // New intersection and new ray:
                    next_ray = Ray3f(its.p, its.toWorld(bsdfRecord.wo));
                    // For RR saying to not continuing with probability of sample()
                    rr_limit = std::min(0.9f, (std::max(fr[0], std::max(fr[1], fr[2]))));
                }
                else {
                    // If it does intersect with an emitter stop the bouncing and add up the contribution weighted with the p_mat_wmat gotten before.
                    keepTracing = false;
                    EmitterQueryRecord emitterRecord(its.mesh->getEmitter(), next_ray.o, its.p, its.shFrame.n, its.uv);
                    // p_mat_wmat gotten from before 
                    // Get p_em_wmat
                    p_em_wmat = its.mesh->getEmitter()->pdf(emitterRecord);
                    w_mat = (p_em_wmat + p_mat_wmat) > FLT_EPSILON ? p_mat_wmat / (p_em_wmat + p_mat_wmat) : 0;

                    Le = its.mesh->getEmitter()->eval(emitterRecord);

                    // If we didn't have bounced in any surface, then we need the whole value of Le to be Lo: w_mat = fr = 1
                    if (n_bounces == 0) {
                        w_mat = 1;
                    }
                    if (measure_last_bsdf == EDiscrete) {
                        // If the material_bsdf is discrete then it's delta so then the pdf would be 0 -> meaning that it wouldn't take it into account
                        // But if it's discrete, it's the only direction that can go to.
                        w_mat = 1;
                    }
                    Lo += fr * Le * w_mat;
                }
            }
            else {
                // If it doesn't intersect stop, and get Background (enviromental light as well)
                keepTracing = false;

                const Emitter* env_emitter = scene->getEnvironmentalEmitter();
                // If we didn't intersect with anything and the scene has a enviromental emitter:
                if (env_emitter) {
                    // Then the background has an emmitter.
                    // do we have to weight this one as well?
                    // TODO HERE
                    EmitterQueryRecord emitterRecord(env_emitter, next_ray.o, its.p, its.shFrame.n, its.uv);
                    // p_mat_wmat gotten from before 
                    // Get p_em_wmat
                    p_em_wmat = env_emitter->pdf(emitterRecord);
                    w_mat = (p_em_wmat + p_mat_wmat) > FLT_EPSILON ? p_mat_wmat / (p_em_wmat + p_mat_wmat) : 0;

                    Le = scene->getBackground(next_ray);
                    // If we didn't have bounced in any surface, then we need the whole value of Le to be Lo: w_mat = fr = 1
                    if (n_bounces == 0) {
                        w_mat = 1;
                    }
                    if (measure_last_bsdf == EDiscrete) {
                        // If the material_bsdf is discrete then it's delta so then the pdf would be 0 -> meaning that it wouldn't take it into account
                        // But if it's discrete, it's the only direction that can go to.
                        w_mat = 1;
                    }

                    Lo += fr * Le * w_mat;
                }// else: !env_emitter
            }

            // Extra end conditions:
            // not continuing with probability of sample() accumulated
            float rnd = (float)rand() / RAND_MAX;
            keepTracing = keepTracing && (rnd < rr_limit || n_bounces < 3); //90% of continuing.
            if (n_bounces >= 3) {
                if (keepTracing) {
                    // Then we continue with probability:
                    fr /= rr_limit;
                }
                else {
                    // We don't continue with probability:
                    fr /= (1 - rr_limit);
                }
            }

            n_bounces += 1;
        }
        return Lo;
    }
    std::string toString() const
    {
        return "Direct Whitted Integrator []";
    }
};
NORI_REGISTER_CLASS(PathTracingMIS, "path_mis");
NORI_NAMESPACE_END