#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectEmitterSampling : public Integrator {
public:
	DirectEmitterSampling(const PropertyList& props) {
	}
protected:

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
		Color3f Lo(0.);
		Color3f sum(0.);

		//Find the surface that is visible in the requested direction
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);

		//Sample randomly a light source
		float pdf_light;
		const Emitter* light = scene->sampleEmitter(sampler->next1D(), pdf_light);
		//Sample the light
		EmitterQueryRecord emitterRecord(its.p);
		Color3f Li = light->sample(emitterRecord, sampler->next2D(), 0.);
		Color3f Le(0.);

		if (its.mesh->isEmitter()) {
			Le = its.mesh->getEmitter()->eval(EmitterQueryRecord(ray.o));
		}

		//Visibility check
		Ray3f sray(its.p, emitterRecord.wi);
		Intersection it_shadow;
		if (scene->rayIntersect(sray, it_shadow)) {
			if (it_shadow.t < (emitterRecord.dist - 1.e-5)) {
				return Le;
			}
		}
		//BSDF 
		BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d),
			its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);

		//Probability of the sample of the point of the light source
		float pdf_light_point = light->pdf(emitterRecord);

	
		//Compute the new normal for bump mapping
		if (its.mesh->getBSDF()->hasDisplacementMap()) {
			float du = 0.001;
			float dv = 0.001;

			//Displacement in u+du,v
			Intersection it_eval = Intersection();
			//it_eval.p = its.p + du * its.shading.dpdu;
			Point2f uv_du = its.uv + Point2f(du, 0);
			if (uv_du[0] >= 1) uv_du[0] = 1;
			//it_eval.shFrame = Frame((Normal3f)(its.shading.dpdu.cross(its.shading.dpdv) + dv * its.shading.dndv).normalized());
			float uDisplace = its.mesh->getBSDF()->displacement(uv_du);
			//std::cout << "Displacement: " << uDisplace << std::endl;
			//Displacement in u,v+dv
			//it_eval.p = its.p + dv * its.shading.dpdv;
			Point2f uv_dv = its.uv + Vector2f(0, dv);
			if (uv_dv[1] >= 1) uv_dv[1] = 1;
			//it_eval.shFrame = Frame((Normal3f)(its.shading.dpdv.cross(its.shading.dpdu) + du * its.shading.dndu).normalized());
			float vDisplace = its.mesh->getBSDF()->displacement(uv_dv);

			// Displace in u,v
			float displace = its.mesh->getBSDF()->displacement(its.uv);

			//Get the new point derivatives
			Vector3f new_dpdu = its.shading.dpdu + (uDisplace - displace) / du * its.shading.n; //+ displace * its.shading.dndu;
			Vector3f new_dpdv = its.shading.dpdv + (vDisplace - displace) / dv * its.shading.n; //+ displace * its.shading.dndv;
			std::cout << "Old dpdu: " << its.shading.dpdu.toString() << std::endl;
			std::cout << "New dpdu: " << new_dpdv.toString() << std::endl;

			its.shading.n = (Normal3f)new_dpdu.cross(new_dpdv).normalized();
			//std::cout << "Previous Normal: " << its.shFrame.n.toString() << std::endl;
			its.shFrame = Frame(its.shading.n);
			//std::cout << "New Normal: " << its.shading.n.toString() << endl;
		}
		
		//Accumulate the sample
		sum = (Li * its.mesh->getBSDF()->eval(bsdfRecord) *
			its.shFrame.n.dot(emitterRecord.wi)) / (pdf_light * pdf_light_point);

		Lo = Le + sum;
		return Lo;
	}


	std::string toString() const {
		return "Direct Emitter Sampling []";
	}
};

NORI_REGISTER_CLASS(DirectEmitterSampling, "direct_ems");
NORI_NAMESPACE_END