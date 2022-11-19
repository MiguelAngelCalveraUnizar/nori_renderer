#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
NORI_NAMESPACE_BEGIN
class DirectMIS : public Integrator
{
public:
    DirectMIS(const PropertyList& props)
    {
        /* No parameters this time */
    }
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
		Color3f Lo(0.); // Total radiance
        Color3f Le(0.); // From the first intersection (If its emitter only)
        Color3f Li_mats(0.); // From the material sampling
		Color3f Li_ems(0.); // From the emmiter sampling
		
		// Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return scene->getBackground(ray);
        }
		
		//**********************************************************
		//Check if the intersected mesh is an emitter and compute Le
		//**********************************************************
		if (its.mesh->isEmitter())
			Le = its.mesh->getEmitter()->eval(EmitterQueryRecord(ray.o));
		
		//***************//
		//Sample emmiter//
		//**************//
		float pdf_light;
		const Emitter* light = scene->sampleEmitter(sampler->next1D(), pdf_light); 	//const Emitter *sampleEmitter(float rnd, float &pdf) const;
		//Sample a point from the light
		EmitterQueryRecord emitterRecordEms(its.p);
		Li_ems = light->sample(emitterRecordEms, sampler->next2D(), 0.);
		//Visibility check
		Ray3f sray(its.p, emitterRecordEms.wi);
		Intersection it_shadow;
		bool isEmmiterVisible = true;
		if (scene->rayIntersect(sray, it_shadow)) {
			if (it_shadow.t < (emitterRecordEms.dist - 1.e-5)) {
				isEmmiterVisible = false;
			}
		}
		if (isEmmiterVisible) {
			//BSDF 
			BSDFQueryRecord bsdfRecordEms(its.toLocal(-ray.d),
				its.toLocal(emitterRecordEms.wi), its.uv, ESolidAngle);

			//Probability of the sample of the point of the light source
			float pdf_light_point = light->pdf(emitterRecordEms);

			//Compute the p_em(sample ems)
			float p_em_wem = pdf_light_point * pdf_light;
			//Accumulate the sample
			Li_ems *=  (its.mesh->getBSDF()->eval(bsdfRecordEms) *
				its.shFrame.n.dot(emitterRecordEms.wi)) / p_em_wem;


			
		}

		//***************//
		//Sample material//
		//**************//
		BSDFQueryRecord bsdfRecordMat(its.toLocal(-ray.d), its.uv);

        // We sample a direction wo with probability proportional to the BSDF. Then we get the fr*cos/p_omega            
        Color3f fr = its.mesh->getBSDF()->sample(bsdfRecordMat, sampler->next2D());
		//Compute the p_mat(sample mats)
		float p_mat_wmat = its.mesh->getBSDF()->pdf(bsdfRecordMat);
        // Ray from p with wo to see if it intersects in a emitter
        Ray3f next_ray(its.p, its.toWorld(bsdfRecordMat.wo));
        Intersection it_next;
        if (scene->rayIntersect(next_ray, it_next)){
            // If it intersects with something, then we check if the intersection is in a emitter.
            if (it_next.mesh->isEmitter()) {
                EmitterQueryRecord emitterRecordMat = EmitterQueryRecord(it_next.mesh->getEmitter(), its.p, it_next.p, it_next.shFrame.n, it_next.uv);
                Li_mats = it_next.mesh->getEmitter()->eval(emitterRecordMat);
                Li_mats = Li_mats * fr;
            }
        }
        else{
            Li_mats = fr * scene->getBackground(next_ray); // For some reason getBackground sometimes gives Nan
        }
	
		//***************************
		//Compute the  result of  MIS
		//***************************
		Lo = Le + Li_ems + Li_mats;
		
		return Lo;
	}
	
	
	std::string toString() const {
		return "Multiple Importance Sampling []";
	}
	
};
NORI_REGISTER_CLASS(DirectMIS, "direct_mis");
NORI_NAMESPACE_END
