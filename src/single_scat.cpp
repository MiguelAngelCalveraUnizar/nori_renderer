#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>


#include <nori/pf.h>

NORI_NAMESPACE_BEGIN
class SingleScat : public Integrator
{
public:
    SingleScat(const PropertyList& props)
    {
        //mu_a = 0.1f;
        //mu_s = 0.6f;
        //mu_t = mu_a + mu_t;
    }
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Color3f Lo(0.); // Total radiance
        Color3f Le(0.); // Emitter radiance

        // Boundary of the medium
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return scene->getBackground(ray);
        }
        // Imagine for some reason we get a participating media through the intersection
        // Then we will have access to the mu_t inside, so we can pass it to Transmittance
        Point3f xz = its.p;
        Point3f x = ray.o;
        Vector3f w = ray.d;

        MediumIntersection medIts;
        medIts.medium = scene->getMedium();
        scene->getMedium()->sampleBetween(x, xz, sampler->next1D(), medIts);
        Le = medIts.medium->Transmittance(x, xz) * DirectLight(scene, sampler, its, medIts, ray);

        // Inscattering
        Lo = Le + medIts.medium->Transmittance(x, medIts.xt) * Inscattering(scene, sampler, medIts, ray) / medIts.pdf_xt;
       
        return Lo;
    }

    const Color3f DirectLight(const Scene* scene, Sampler* sampler, Intersection its, MediumIntersection medIts, Ray3f ray)const {
        // For readability we turn its into xz etc
        //std::cout << "DirectLight in single scat\n";
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
        
        BSDFQueryRecord bsdfRecordEms(its.toLocal(-ray.d),
            its.toLocal(emitterRecordEms.wi), its.uv, ESolidAngle);

        if (Visibility) {
            //Compute the p_em(sample ems) and p_mat(sample ems)
            p_em_wem = pdf_light_point * pdf_light;
            // For MSI we need to evaluate respect to p_mat_wem the pdf of the direction
            p_mat_wem = its.mesh->getBSDF()->pdf(bsdfRecordEms);

            //Lems = Le * Transmittance(xz, xe) * xz.BRDF.eval(w, (xe - xz)) * cos(xe - xz, xz.n);
            Lems = Le * medIts.medium->Transmittance(xz, xe) * its.mesh->getBSDF()->eval(bsdfRecordEms) *
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
                // Lmat = xem.emit(xz) * Transmittance(xz, xem) * fs;
                Lmat = it_next.mesh->getEmitter()->eval(emitterRecordMat) * fs * medIts.medium->Transmittance(xz, xem);

                // Get p_em_wmat
                p_em_wmat = it_next.mesh->getEmitter()->pdf(emitterRecordMat);
                //Compute the p_mat(sample mats)
                p_mat_wmat = its.mesh->getBSDF()->pdf(bsdfRecordMat);
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

    Color3f Inscattering(const Scene* scene, Sampler* sampler, MediumIntersection its, Ray3f ray) const {
        Point3f xt = its.xt;
        Vector3f w = ray.d;
        Color3f Lems = 0;
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
        PFQueryRecord phaseRecordEms(its.toLocal(-ray.d), its.toLocal(emitterRecordEms.wi));

        if (Visibility) {
            //Compute the p_em(sample ems) and p_mat(sample ems)
            p_em_wem = pdf_light_point * pdf_light;
            // For MSI we need to evaluate respect to p_mat_wem the pdf of the direction
            p_mat_wem = its.medium->getPhaseFunction()->pdf(phaseRecordEms);

            //Lems = Le * Transmittance(xt, xe) * xt.PF.eval(w, (xe - xt)) * mu_s;
            Lems = Le * its.medium->Transmittance(xt, xe) * its.medium->getPhaseFunction()->eval(phaseRecordEms) * its.medium->getScatteringCoeficient() / p_em_wem;;
        }
        
        // Sample the phase function
        //<fs, wo, pdf_m> = xt.PF.sample(w);
        //PFQueryRecord phaseRecordMats(its.toLocal(-ray.d), its.uv);
        PFQueryRecord phaseRecordMats(its.toLocal(-ray.d));
        Color3f fs = its.medium->getPhaseFunction()->sample(phaseRecordMats, sampler->next2D());
        // sample will get us wo and fs
        Vector3f wo = phaseRecordMats.wo;


        // xem = scene.intersect(Ray(xt, wo));
        // Ray from xt with wo to see if it intersects in a emitter
        //if (xem.esEmitter())
        Ray3f next_ray(its.xt, its.toWorld(phaseRecordMats.wo));
        Intersection it_next;
        if (scene->rayIntersect(next_ray, it_next)) {
            if (it_next.mesh->isEmitter()) {
                //xem = scene.intersect(Ray(xz,wo));
                Point3f xem = it_next.p;

                //Lmat = xem.emit(xt) * Transmittance(xt, xem) * fs * mu_s;
                EmitterQueryRecord emitterRecordMat(it_next.mesh->getEmitter(), its.xt, it_next.p, it_next.shFrame.n, it_next.uv);
                Lmat = it_next.mesh->getEmitter()->eval(emitterRecordMat) * fs * its.medium->Transmittance(xt, xem) * its.medium->getScatteringCoeficient();

                // Get p_em_wmat
                p_em_wmat = it_next.mesh->getEmitter()->pdf(emitterRecordMat);
                //Compute the p_mat(sample mats)
                p_mat_wmat = its.medium->getPhaseFunction()->pdf(phaseRecordMats);
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

        return Lems * w_em + Lmat * w_mat;
    }

    std::string toString() const
    {
        return "Single Scattering Integrator []";
    }

    //// This shouldn't be here but whatever
    //float mu_t;
    //float mu_a;
    //float mu_s;
};
NORI_REGISTER_CLASS(SingleScat,"single_scat");
NORI_NAMESPACE_END