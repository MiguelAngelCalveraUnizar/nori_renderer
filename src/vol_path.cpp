#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>


#include <nori/pf.h>

NORI_NAMESPACE_BEGIN
class VolPathIntegrator : public Integrator
{
public:
    VolPathIntegrator(const PropertyList& props)
    {

    }
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Color3f Lo(0.); // Total radiance
        Color3f Ld(0.); // Direct Light to a point.
        Color3f Ls(0.); // Scattering light from the medium to a point
        Color3f Le(0.); // Emitter radiance
		Color3f fr(1); //Thoughput
        float w_mat, p_mat_wmat = 0, p_em_wmat;
		bool keepTracing = true;
		Ray3f next_ray(ray);
		EMeasure measure_last_bsdf = EUnknownMeasure;
		// RR:
        float rr_limit = 0.9f;
        size_t n_bounces = 0;
		
		
		while(keepTracing && fr.getLuminance() > 0) { // If it won't give light stop 
			Le = Color3f(0.); 
			
			//Try to intersect the last generated ray
			Intersection its;
            
            
            bool Intersected = scene->rayIntersect(next_ray, its);
            //bool notIntersected = !Intersected;
            bool intersectedWithEmitter = true;
            bool intersectedWithNonEmitter = false;

            if (Intersected) {
                // If it intersects
                intersectedWithEmitter = its.mesh->isEmitter();
                intersectedWithNonEmitter = !intersectedWithEmitter;
                if (intersectedWithEmitter) {
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

            if(intersectedWithNonEmitter || !Intersected){
                // If it is not a emitter or we didn't intersect
                // Check if next_ray travels thought a participating media
                Vector3f w = next_ray.d;
                MediumIntersection medIts;
                medIts.o = next_ray.o;
                if (Intersected) {
                    medIts.p = its.p;
                }
                else {
                    // If not found intersection, we set it at infinity
                    medIts.p = Point3f(FLT_MAX);
                }
                
                bool mediumFound = scene->rayIntersectMedium(next_ray, medIts);
                bool sampledInsideMedium = false;
                if (mediumFound) {
                    // If there medium -> We have to choose between DirectLight or Inscattering
                    // Now medIts has .medium and information about intersection
                    scene->getMedium()->sampleBetween(sampler->next1D(), medIts);
                    // Now in medIts.xt we have a value between x (start of medium) and infinity
                    float t = medIts.distT; //(medIts.xt - medIts.x).norm();
                    float z = medIts.distZ;//(medIts.xt - medIts.p).norm();

                    sampledInsideMedium = (t >= z); // Sampling outside of the medium->inside a mesh which means doing DirectLight
                        
                    fr *= medIts.medium->Transmittance(medIts.x, medIts.xt) / medIts.prob;
                }
                if (sampledInsideMedium && mediumFound) {
                    // We sampled in a medium:
                    Lo += fr * Inscattering(scene, sampler, its, medIts, next_ray);
                    // Generate next ray
                    // As we hit a medium then we do PF sampling.
                    Vector3f wi = -next_ray.d;
                    PFQueryRecord pfRecord(medIts.toLocal(wi));
                    fr *= scene->getMedium()->getPhaseFunction()->sample(pfRecord, sampler->next2D());
                    measure_last_bsdf = ESolidAngle; //We don't have to treat it as a Discrete PDF
                    //Compute the p_mat(sample mats)
                    p_mat_wmat = medIts.medium->getPhaseFunction()->pdf(pfRecord);
                    // New ray :
                    next_ray = Ray3f(medIts.xt, its.toWorld(pfRecord.wo));

                }
                else {
                    if (intersectedWithNonEmitter) {
                        // We hit a surface that is not a emitter: (if there's medium fr was already updated)
                        Lo += fr * DirectLight(scene, sampler, its, medIts, next_ray);
                        //Generate next ray
                        // As the surface was hitted we do BSDF
                        // Sample the BRDF
                        Vector3f wi = -next_ray.d;
                        BSDFQueryRecord bsdfRecordMat(its.toLocal(wi), its.uv);
                        fr *= its.mesh->getBSDF()->sample(bsdfRecordMat, sampler->next2D());
                        measure_last_bsdf = bsdfRecordMat.measure;
                        //Compute the p_mat(sample mats)
                        p_mat_wmat = its.mesh->getBSDF()->pdf(bsdfRecordMat);

                        // New ray:
                        next_ray = Ray3f(its.p, its.toWorld(bsdfRecordMat.wo));
                    }

                    if (!Intersected) {
                        // There's no medium or we didn't had medium, then normal treatment
                        // Which means stop and add background
                        keepTracing = false;
                        const Emitter* env_emitter = scene->getEnvironmentalEmitter();
                        if (env_emitter) {
                            // Then the background has an emmitter.
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
                        }
                    }
                }
            }
			// Extra end conditions:
            // For RR saying to not continuing with probability of sample()
            rr_limit = std::min(0.9f, (std::max(fr[0], std::max(fr[1], fr[2]))));
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
	
	
    /* 
    * Sample a light for a direct light to the point in intersection (xz). 
    * It assumes a its.p is a intersection to a material
    * We DON'T get the next direction to sample, that has to be done separatly
    */
    const Color3f DirectLight(const Scene* scene, Sampler* sampler, Intersection its, MediumIntersection medIts, Ray3f ray)const {
        // For readability we turn its into xz etc
        Point3f xz = medIts.p;
        Vector3f w = ray.d;
        Color3f Lems(0.);
        float p_em_wem = 0;
        float p_mat_wem = 0;

        // Sample the emitter
        float pdf_light;
        const Emitter* light = scene->sampleEmitter(sampler->next1D(), pdf_light);

        //Sample the light
        EmitterQueryRecord emitterRecordEms(xz);
        Color3f Le = light->sample(emitterRecordEms, sampler->next2D(), 0.);
        float pdf_light_point = light->pdf(emitterRecordEms);
        Point3f xe = emitterRecordEms.p;

        //if (scene.isVisible(xe, xz))
        Ray3f sray(xz, emitterRecordEms.wi);
        Intersection it_shadow;
        bool Visibility = true;
        if (scene->rayIntersect(sray, it_shadow)) {
            if (it_shadow.t < (emitterRecordEms.dist - 1.e-5)) {
                //Then there's no visibility
                Visibility = false;
            }
        }

        //BSDF
        BSDFQueryRecord bsdfRecordEms(its.toLocal(-ray.d),
            its.toLocal(emitterRecordEms.wi), its.uv, ESolidAngle);

        if (Visibility) {
            //Compute the p_em(sample ems) and p_mat(sample ems)
            p_em_wem = pdf_light_point * pdf_light;
            // For MSI we need to evaluate respect to p_mat_wem the pdf of the direction
            p_mat_wem = its.mesh->getBSDF()->pdf(bsdfRecordEms);

            float w_em = 0;
            if ((p_em_wem + p_mat_wem) > FLT_EPSILON) {
                w_em = 1 / (p_em_wem + p_mat_wem);
            }

            //Lems = Le * Transmittance(xz, xe) * xz.BRDF.eval(w, (xe - xz)) * cos(xe - xz, xz.n);
            // But wait! We are not sure that the light is also in the medium or even that p is in the medium!
            // So we can create a mediumIntersection to handle that:
            MediumIntersection medIts_em;
            medIts_em.o = xz; // Point in the intersection
            medIts_em.p = xe; // Point in emitter
            bool mediumFound_em = scene->rayIntersectMedium(sray, medIts_em);
            // Now we have if there's a medium in the way of the light we put the correct Transmittance:
            float Transmittance_em = 1;
            if (mediumFound_em) {
                Transmittance_em = medIts_em.medium->Transmittance(medIts_em.x, medIts_em.xz);
            }
            // This has to be correctly weighted
            Lems = w_em * Le * Transmittance_em * its.mesh->getBSDF()->eval(bsdfRecordEms) *
                its.shFrame.n.dot(emitterRecordEms.wi);
        }

        return Lems;
    }

    /*
    * Sample a light for a direct light to the point in MediumIntersection (xz). 
    * It assumes a its.p is a intersection to a medium, and that MediumIntersection is correctly defined
    * We DON'T get the next direction to sample, that has to be done separatly
    */
    const Color3f Inscattering(const Scene* scene, Sampler* sampler, Intersection its, MediumIntersection medIts, Ray3f ray)const {
        // For readability we turn its into xz etc
        Point3f xt = medIts.xt;
        Vector3f w = ray.d;
        Color3f Lems = 0;
        Color3f Lmat = 0;
        float p_em_wem = 0;
        float p_mat_wem = 0;

        // Sample the emitter
        float pdf_light;
        const Emitter* light = scene->sampleEmitter(sampler->next1D(), pdf_light);

        //Sample the light
        EmitterQueryRecord emitterRecordEms(xt);
        Color3f Le = light->sample(emitterRecordEms, sampler->next2D(), 0.);
        float pdf_light_point = light->pdf(emitterRecordEms);
        Point3f xe = emitterRecordEms.p;

        //if (scene.isVisible(xe, xz))
        Ray3f sray(xt, emitterRecordEms.wi);
        Intersection it_shadow;
        int Visibility = 1;
        if (scene->rayIntersect(sray, it_shadow)) {
            if (it_shadow.t < (emitterRecordEms.dist - 1.e-5)) {
                //Then there's no visibility
                Visibility = 0;
            }
        }

        //PFQueryRecord phaseRecordEms(its.toLocal(-ray.d), its.toLocal(emitterRecordEms.wi), its.uv, ESolidAngle); //
        PFQueryRecord phaseRecordEms(medIts.toLocal(-ray.d), medIts.toLocal(emitterRecordEms.wi));

        if (Visibility) {
            //Compute the p_em(sample ems) and p_mat(sample ems)
            p_em_wem = pdf_light_point * pdf_light;

            // For MSI we need to evaluate respect to p_mat_wem the pdf of the direction
            // WARNING: GetPhaseFuntion will ned a phaseRecordEms that is correctly defined
            p_mat_wem = medIts.medium->getPhaseFunction()->pdf(phaseRecordEms);

            float w_em = 0;
            if ((p_em_wem + p_mat_wem) > FLT_EPSILON) {
                w_em = 1 / (p_em_wem + p_mat_wem);
            }

            //Lems = Le * Transmittance(xz, xe) * xz.BRDF.eval(w, (xe - xz)) * cos(xe - xz, xz.n);
            // But wait! We are not sure that the light is also in the medium or even that p is in the medium!
            // So we can create a mediumIntersection to handle that:
            MediumIntersection medIts_em;
            medIts_em.o = xt; // Point in the intersection
            medIts_em.p = xe; // Point in emitter
            bool mediumFound_em = scene->rayIntersectMedium(sray, medIts_em);
            // Now we have if there's a medium we put the correct Transmittance:
            float Transmittance_em = 1;
            if (mediumFound_em) {
                Transmittance_em = medIts_em.medium->Transmittance(medIts_em.x, medIts_em.xz);
            }
            Lems = w_em * Le * Transmittance_em * medIts.medium->getPhaseFunction()->eval(phaseRecordEms) *
                medIts.medium->getScatteringCoeficient();
        }

        return Lems;
    }

    std::string toString() const
    {
        return "Volumetric Path Integrator []";
    }

};
NORI_REGISTER_CLASS(VolPathIntegrator,"vol_path_integrator");
NORI_NAMESPACE_END
