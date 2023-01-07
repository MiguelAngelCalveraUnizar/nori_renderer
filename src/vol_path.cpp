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
			
            if (scene->rayIntersect(next_ray, its)) {
                // If it intersect but it's not an emitter create another bounce with some prob.
                if (!its.mesh->isEmitter()) {
                    // Check if next_ray travels thought a participating media
                    Vector3f w = next_ray.d;
                    MediumIntersection medIts;
                    medIts.o = next_ray.o;
                    medIts.p = its.p;
                    bool mediumFound = scene->rayIntersectMedium(next_ray, medIts);
                    if (!mediumFound) {
                        Lo += fr * EmsSampling(scene, sampler, its, medIts, next_ray);
                        //std::cout << "Lo value: " << Lo.toString() << std::endl;
                        // Ls is 0 (No medium -> no inscattering and transmittance = 1)

                        // Sample the BRDF
                        //<fs, wo, pdf_m> = xz.BRDF.sample(w);
                        BSDFQueryRecord bsdfRecordMat(its.toLocal(-next_ray.d), its.uv);
                        fr *= its.mesh->getBSDF()->sample(bsdfRecordMat, sampler->next2D());
                        measure_last_bsdf = bsdfRecordMat.measure;
                        //Compute the p_mat(sample mats)
                        p_mat_wmat = its.mesh->getBSDF()->pdf(bsdfRecordMat);

                        // New ray:
                        next_ray = Ray3f(its.p, its.toWorld(bsdfRecordMat.wo));
                        // For RR saying to not continuing with probability of sample()
                        rr_limit = std::min(0.9f, (std::max(fr[0], std::max(fr[1], fr[2]))));
                    }
                    else {
                        
                        // Now medIts has .medium and information about intersection
                        scene->getMedium()->sampleBetween(sampler->next1D(), medIts);
                        fr *= medIts.medium->Transmittance(medIts.x, medIts.xt) / medIts.pdf_xt;
                        Lo += fr * EmsSampling(scene, sampler, its, medIts, next_ray);
                        //Generate next ray
                        Vector3f wi = -next_ray.d;
                        PFQueryRecord pfRecord(medIts.toLocal(wi));
                        Color3f  phase = scene->getMedium()->getPhaseFunction()->sample(pfRecord, sampler->next2D());
                        Vector3f wo = pfRecord.wo;
                        next_ray = Ray3f(its.p, its.toWorld(pfRecord.wo));

                        
                        /*if (isnan(Ls[0]) || isnan(Ls[1]) || isnan(Ls[2])) {
                            std::cout << "Ls is nan \n";
                        }*/
                        
                    }

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
                }
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

    const Color3f DirectLight(const Scene* scene, Sampler* sampler, Intersection its, MediumIntersection medIts, Ray3f ray)const {
        // For readability we turn its into xz etc
        Point3f xz = its.p;
        Vector3f w = ray.d;
        Color3f Lems = 0;
        Color3f Lmat = 0;
        float p_em_wem = 0;
        float p_mat_wem = 0;
        float p_mat_wmat = 0;
        float p_em_wmat = 0;

        // Sample the emitter
        //<E, pdf_E> = scene.sampleEmiter(xz)
        float pdf_light;
        const Emitter* light = scene->sampleEmitter(sampler->next1D(), pdf_light);

        //Sample the light
        //< Le, xe, pdf_e > = E.sample(xz);
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

            //Lems = Le * Transmittance(xz, xe) * xz.BRDF.eval(w, (xe - xz)) * cos(xe - xz, xz.n);
            // But wait! We are not sure that the light is also in the medium or even that p is in the medium!
            // So we can create a mediumIntersection to handle that:
            MediumIntersection medIts_em;
            medIts_em.o = xz; // Point in the intersection
            medIts_em.p = xe; // Point in emitter
            bool mediumFound_em = scene->rayIntersectMedium(sray, medIts_em);
            // Now we have if there's a medium we put the correct Transmittance:
            float Transmittance_em = 1;
            if (mediumFound_em) {
                Transmittance_em = medIts_em.medium->Transmittance(medIts_em.x, medIts_em.xz);
            }

            Lems = Le * Transmittance_em * its.mesh->getBSDF()->eval(bsdfRecordEms) *
                its.shFrame.n.dot(emitterRecordEms.wi) / p_em_wem;
        }

        // Sample the BRDF
        //<fs, wo, pdf_m> = xz.BRDF.sample(w);
        BSDFQueryRecord bsdfRecordMat(its.toLocal(-ray.d), its.uv);
        Color3f fs = its.mesh->getBSDF()->sample(bsdfRecordMat, sampler->next2D());
        Vector3f wo = bsdfRecordMat.wo;

        //xem = scene.intersect(Ray(xz, wo));
        // Ray from p with wo to see if it intersects in a emitter
        //if (xem.esEmitter())
        Ray3f next_ray(its.p, its.toWorld(bsdfRecordMat.wo));
        Intersection it_next;
        if (scene->rayIntersect(next_ray, it_next)) {
            if (it_next.mesh->isEmitter()) {
                //xem = scene.intersect(Ray(xz,wo));
                Point3f xem = it_next.p;
                EmitterQueryRecord emitterRecordMat(it_next.mesh->getEmitter(), its.p, it_next.p, it_next.shFrame.n, it_next.uv);
                
                // Get p_em_wmat
                p_em_wmat = it_next.mesh->getEmitter()->pdf(emitterRecordMat);
                //Compute the p_mat(sample mats)
                p_mat_wmat = its.mesh->getBSDF()->pdf(bsdfRecordMat);


                MediumIntersection medIts_mats;
                medIts_mats.o = xz; // Point in the intersection
                medIts_mats.p = xem; // Point in emitter
                bool mediumFound_mats = scene->rayIntersectMedium(next_ray, medIts_mats);
                // Now we have if there's a medium we put the correct Transmittance:
                float Transmittance_mats = 1;
                if (mediumFound_mats) {
                    Transmittance_mats = medIts_mats.medium->Transmittance(medIts_mats.x, medIts_mats.xz);
                }

                // Lmat = xem.emit(xz) * Transmittance(xz, xem) * fs;
                Lmat = it_next.mesh->getEmitter()->eval(emitterRecordMat) * Transmittance_mats * fs;
            }
        }
        /*else {
            // Here add support for enviromental lights if needed.
        }*/

        // MIS both contributions
        //return Lems / (pdf_e + E.pdf(xem)) + Lmat / (pdf_m + xz.BRDF.pdf(w, (xe - xz));
        //Compute the weights
        float w_em = 0;
        float w_mat = 0;
        if ((p_em_wem + p_mat_wem) > FLT_EPSILON) {
            w_em = p_em_wem / (p_em_wem + p_mat_wem);
            // Lems was divided by p_em_wem before, so now it multiplies
        }
        if ((p_em_wmat + p_mat_wmat) > FLT_EPSILON) {
            w_mat = p_mat_wmat / (p_em_wmat + p_mat_wmat);
            // Lmats was divided by p_mat_wmat before (fs contains already p_mat_wmat), so now it is multiplied.
        }

        return Lems * w_em + Lmat * w_mat;
    }
	
	const Color3f EmsSampling(const Scene* scene, Sampler* sampler, Intersection its, MediumIntersection medIts, Ray3f ray)const {
        // For readability we turn its into xz etc
        Point3f xz = its.p;
        Vector3f w = ray.d;
        Color3f Lems(0.);
        //Color3f Lmat = 0;
        float p_em_wem = 0;
        float p_mat_wem = 0;
        //float p_mat_wmat = 0;
        //float p_em_wmat = 0;

        // Sample the emitter
        //<E, pdf_E> = scene.sampleEmiter(xz)
        float pdf_light;
        const Emitter* light = scene->sampleEmitter(sampler->next1D(), pdf_light);

        //Sample the light
        //< Le, xe, pdf_e > = E.sample(xz);
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

            if (p_em_wem < FLT_EPSILON) {
                p_em_wem = FLT_EPSILON;
            }

			float w_em = 0;
			if ((p_em_wem + p_mat_wem) > FLT_EPSILON) {
				w_em = p_em_wem / (p_em_wem + p_mat_wem);
			}

            //Lems = Le * Transmittance(xz, xe) * xz.BRDF.eval(w, (xe - xz)) * cos(xe - xz, xz.n);
            // But wait! We are not sure that the light is also in the medium or even that p is in the medium!
            // So we can create a mediumIntersection to handle that:
            MediumIntersection medIts_em;
            medIts_em.o = xz; // Point in the intersection
            medIts_em.p = xe; // Point in emitter
            bool mediumFound_em = scene->rayIntersectMedium(sray, medIts_em);
            // Now we have if there's a medium we put the correct Transmittance:
            float Transmittance_em = 1;
            if (mediumFound_em) {
                Transmittance_em = medIts_em.medium->Transmittance(medIts_em.x, medIts_em.xz);
            }
            //std::cout << "Transmitance value: " << Transmittance_em << std::endl;
            Lems = Le * Transmittance_em * its.mesh->getBSDF()->eval(bsdfRecordEms) *
                its.shFrame.n.dot(emitterRecordEms.wi) / p_em_wem;
		}


        /*else {
            // Here add support for enviromental lights if needed.
        }*/
		
        return Lems;
    }

    Color3f Inscattering(const Scene* scene, Sampler* sampler, MediumIntersection medIts, Ray3f ray) const {
        Point3f xt = medIts.xt;
        Vector3f w = ray.d;
        Color3f Lems(0.);
        Color3f Lmat = 0;
        float p_em_wem = 0;
        float p_mat_wem = 0;
        float p_mat_wmat = 0;
        float p_em_wmat = 0;

        // Sample the emitter
        //<E, pdf_E> = scene.sampleEmiter(xt)
        float pdf_light;
        const Emitter* light = scene->sampleEmitter(sampler->next1D(), pdf_light);
        //Sample the light
        //    < Le, xe, pdf_e > = E.sample(xt);
        EmitterQueryRecord emitterRecordEms(xt);
        Color3f Le = light->sample(emitterRecordEms, sampler->next2D(), 0.);
        float pdf_light_point = light->pdf(emitterRecordEms);
        Point3f xe = emitterRecordEms.p;

        //if (scene.isVisible(xe, xt))
        Ray3f sray(xt, emitterRecordEms.wi);
        Intersection it_shadow;
        bool Visibility = true;
        if (scene->rayIntersect(sray, it_shadow)) {
            if (it_shadow.t < (emitterRecordEms.dist - 1.e-5)) {
                //Then there's no visibility
                Visibility = false;
            }
        }
        //PFQueryRecord phaseRecordEms(its.toLocal(-ray.d), its.toLocal(emitterRecordEms.wi), its.uv, ESolidAngle); //
        PFQueryRecord phaseRecordEms(medIts.toLocal(-ray.d), medIts.toLocal(emitterRecordEms.wi));

        if (Visibility) {
            //Compute the p_em(sample ems) and p_mat(sample ems)
            p_em_wem = pdf_light_point * pdf_light;
            if (p_em_wem < FLT_EPSILON) {
                p_em_wem = FLT_EPSILON;
            }
            // For MSI we need to evaluate respect to p_mat_wem the pdf of the direction
            // WARNING: GetPhaseFuntion will ned a phaseRecordEms that is correctly defined
            p_mat_wem = medIts.medium->getPhaseFunction()->pdf(phaseRecordEms);

            MediumIntersection medIts_em;
            medIts_em.o = xt; // Point in the intersection
            medIts_em.p = xe; // Point in emitter
            bool mediumFound_em = scene->rayIntersectMedium(sray, medIts_em);
            // Now we have if there's a medium we put the correct Transmittance:
            float Transmittance_em = 1;
            if (mediumFound_em) {
                Transmittance_em = medIts_em.medium->Transmittance(medIts_em.x, medIts_em.xz);
            }

            //Lems = Le * Transmittance(xt, xe) * xt.PF.eval(w, (xe - xt)) * mu_s;
            // We only check for medium in Transmittance cause in xt we assured before there's a medium
            // That's also why there we use medIts. The phase function and scattering coeficient are important for
            Lems = Le * Transmittance_em * medIts.medium->getPhaseFunction()->eval(phaseRecordEms) * medIts.medium->getScatteringCoeficient() / p_em_wem;

            if (isnan(Lems[0]) || isnan(Lems[1]) || isnan(Lems[2])) {
                std::cout << "Lems is nan \n";
                std::cout << "Le " << Le.toString() << " ; pf_eval " << medIts.medium->getPhaseFunction()->eval(phaseRecordEms).toString();
                std::cout << " ; mu_s " << medIts.medium->getScatteringCoeficient() << " ; p_em_wem " << p_em_wem;
                std::cout << "pdf_light " << pdf_light << " ; pdf_light_point "<< pdf_light_point<<"\n";
            }

        }
        
        // Sample the phase function
        //<fs, wo, pdf_m> = xt.PF.sample(w);
        //PFQueryRecord phaseRecordMats(its.toLocal(-ray.d), its.uv);
        PFQueryRecord phaseRecordMats(medIts.toLocal(-ray.d));
        Color3f fs = medIts.medium->getPhaseFunction()->sample(phaseRecordMats, sampler->next2D());
        // sample will get us wo and fs
        Vector3f wo = phaseRecordMats.wo;


        // xem = scene.intersect(Ray(xt, wo));
        // Ray from xt with wo to see if it intersects in a emitter
        //if (xem.esEmitter())
        Ray3f next_ray(medIts.xt, medIts.toWorld(phaseRecordMats.wo));
        Intersection it_next;
        if (scene->rayIntersect(next_ray, it_next)) {
            if (it_next.mesh->isEmitter()) {
                //xem = scene.intersect(Ray(xz,wo));
                Point3f xem = it_next.p;
                //Lmat = xem.emit(xt) * Transmittance(xt, xem) * fs * mu_s;
                EmitterQueryRecord emitterRecordMat(it_next.mesh->getEmitter(), medIts.xt, it_next.p, it_next.shFrame.n, it_next.uv);

                // Get p_em_wmat
                p_em_wmat = it_next.mesh->getEmitter()->pdf(emitterRecordMat);
                //Compute the p_mat(sample mats)
                p_mat_wmat = medIts.medium->getPhaseFunction()->pdf(phaseRecordMats);

                MediumIntersection medIts_mats;
                medIts_mats.o = xt; // Point in the intersection
                medIts_mats.p = xem; // Point in emitter
                bool mediumFound_mats = scene->rayIntersectMedium(next_ray, medIts_mats);
                // Now we have if there's a medium we put the correct Transmittance:
                float Transmittance_mats = 1;
                if (mediumFound_mats) {
                    Transmittance_mats = medIts_mats.medium->Transmittance(medIts_mats.x, medIts_mats.xz);
                }

                Lmat = it_next.mesh->getEmitter()->eval(emitterRecordMat) * fs * Transmittance_mats * medIts.medium->getScatteringCoeficient();
            }
        }
        /*else {
            // Here add support for enviromental lights if needed.
        }*/


        // MIS both contributions
        /*return Lems / (pdf_e + E.pdf(xem))
            + Lpf / (pdf_m + xt.PF.pdf(w, (xe - xt))*/

        //Compute the weights
        float w_em = 0;
        float w_mat = 0;
        if ((p_em_wem + p_mat_wem) > FLT_EPSILON) {
            w_em = p_em_wem / (p_em_wem + p_mat_wem);
            // Lems was divided by p_em_wem before, so now it multiplies
        }
        if ((p_em_wmat + p_mat_wmat) > FLT_EPSILON) {
            w_mat = p_mat_wmat / (p_em_wmat + p_mat_wmat);
            // Lmats was divided by p_mat_wmat before (fs contains already p_mat_wmat), so now it is multiplied.
        }


        if (isnan(Lems[0])|| isnan(Lems[1])|| isnan(Lems[2])) {
            std::cout << "Lems is nan \n";
        }

        return Lems * w_em + Lmat * w_mat;
    }

    std::string toString() const
    {
        return "Volumetric Path Integrator []";
    }

};
NORI_REGISTER_CLASS(VolPathIntegrator,"vol_path_integrator");
NORI_NAMESPACE_END