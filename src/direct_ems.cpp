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
			its.shading.n = its.shFrame.n + its.mesh->getBSDF()->displacement(its.uv);
			its.shFrame = Frame(its.shading.n);

		}

		//Accumulate the sample
		sum = (Li * its.mesh->getBSDF()->eval(bsdfRecord) *
			its.shFrame.n.dot(emitterRecord.wi)) / (pdf_light * pdf_light_point);
		sum = sum.clamp();
		Lo = Le + sum;
		return Lo;
	}


	std::string toString() const {
		return "Direct Emitter Sampling []";
	}
};

NORI_REGISTER_CLASS(DirectEmitterSampling, "direct_ems");
NORI_NAMESPACE_END