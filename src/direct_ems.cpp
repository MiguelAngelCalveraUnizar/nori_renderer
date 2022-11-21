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
		bool computeRadiance = true;
		//Find the surface that is visible in the requested direction
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);

		//Sample randomly a light source
		float pdf_light;
		const Emitter* light = scene->sampleEmitter(sampler->next1D(), pdf_light); 	//const Emitter *sampleEmitter(float rnd, float &pdf) const;
		//Sample the light
		EmitterQueryRecord emitterRecord(its.p);
		Color3f Li = light->sample(emitterRecord, sampler->next2D(), 0.);
		Color3f Le(0.);
		//Visibility check
		Ray3f sray(its.p, emitterRecord.wi);
		Intersection it_shadow;
		if (scene->rayIntersect(sray, it_shadow)) {
			if (it_shadow.t < (emitterRecord.dist - 1.e-5)) {
				computeRadiance = false;
			}
		}
		if (its.mesh->isEmitter())
			Le = its.mesh->getEmitter()->eval(EmitterQueryRecord(ray.o));
		if (computeRadiance) {
			//BSDF 
			BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d),
				its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);

			//Probability of the sample of the point of the light source
			float pdf_light_point = light->pdf(emitterRecord);

			//Accumulate the sample
			sum = (Li * its.mesh->getBSDF()->eval(bsdfRecord) *
				its.shFrame.n.dot(emitterRecord.wi)) / (pdf_light * pdf_light_point);



		}
		Lo = Le + sum;
		return Lo;
	}


	std::string toString() const {
		return "Direct Emitter Sampling []";
	}
};

NORI_REGISTER_CLASS(DirectEmitterSampling, "direct_ems");
NORI_NAMESPACE_END