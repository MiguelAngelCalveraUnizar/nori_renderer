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

    }
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Color3f Lo(0.); // Total radiance
        Color3f Ld(0.); // Direct Light to a point.
        Color3f Ls(0.); // Scattering light from the medium to a point
        Color3f Le(0.); // Emitter radiance

        // Boundary of the medium
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            // This is incorrect, the background is also affected by transmittance and inscattering
            return scene->getBackground(ray);
        }
        // Get value of the emmiter if it is one.
        if (its.mesh->isEmitter()) {
            Le = its.mesh->getEmitter()->eval(EmitterQueryRecord(its.mesh->getEmitter(), ray.o, its.p, its.shFrame.n, its.uv));
        }

        Vector3f w = ray.d;
        MediumIntersection medIts;
        medIts.o = ray.o;
        medIts.p = its.p;
        bool mediumFound = scene->rayIntersectMedium(ray, medIts);
        if (!mediumFound) {
            Ld = DirectLight(scene, sampler, its, medIts, ray);
            Lo = Le + Ld; // Ls is 0 (No medium -> no inscattering and transmittance = 1)
            return Lo;
        }

        // Now medIts has .medium and information about intersection
        scene->getMedium()->sampleBetween(sampler->next1D(), medIts);
        bool sampledInsideMedium = false;
        float t = medIts.distT; //(medIts.xt - medIts.x).norm();
        float z = medIts.distZ;//(medIts.xt - medIts.xz).norm();
        // Sampling outside of the medium->inside a mesh which means doing DirectLight
        //std::cout << " z: " << z << " t" << t<<"\n";
        sampledInsideMedium = ((t-z) < FLT_EPSILON); 

        if (sampledInsideMedium) {
            // Inscattering
            Ls = medIts.medium->Transmittance(medIts.x, medIts.xt) * Inscattering(scene, sampler, medIts, ray) / medIts.prob;
        }
        else {
            Ld = medIts.medium->Transmittance(medIts.x, medIts.xz) *  DirectLight(scene, sampler, its, medIts, ray)/medIts.prob;
        }

        // We sum everything (emitter and Direct light are affected by same transmittance)
        Lo = Ls + Ld + Le * medIts.medium->Transmittance(medIts.x, medIts.xz);

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

    Color3f Inscattering(const Scene* scene, Sampler* sampler, MediumIntersection medIts, Ray3f ray) const {
        Point3f xt = medIts.xt;
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
        return "Single Scattering Integrator []";
    }

};
NORI_REGISTER_CLASS(SingleScat,"single_scat");
NORI_NAMESPACE_END