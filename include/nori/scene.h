/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    v1 - Dec 01 2020
    Copyright (c) 2020 by Adrian Jarabo

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <nori/accel.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Main scene data structure
 *
 * This class holds information on scene objects and is responsible for
 * coordinating rendering jobs. It also provides useful query routines that
 * are mostly used by the \ref Integrator implementations.
 */
class Scene : public NoriObject {
public:
    /// Construct a new scene object
    Scene(const PropertyList &);

    /// Release all memory
    virtual ~Scene();

    /// Return a pointer to the scene's kd-tree
    const Accel *getAccel() const { return m_accel; }

    /// Return a pointer to the scene's integrator
    const Integrator *getIntegrator() const { return m_integrator; }

    /// Return a pointer to the scene's integrator
    Integrator *getIntegrator() { return m_integrator; }

    /// Return a pointer to the scene's camera
    const Camera *getCamera() const { return m_camera; }

    /// Return a pointer to the scene's sample generator (const version)
    const Sampler *getSampler() const { return m_sampler; }

    /// Return a pointer to the scene's sample generator
    Sampler *getSampler() { return m_sampler; }

    /// Return a reference to an array containing all meshes
    const std::vector<Mesh *> &getMeshes() const { return m_meshes; }

	/// Return a reference to an array containing all lights
	const std::vector<Emitter *> &getLights() const { return m_emitters; }

	/// Return a the scene background
	Color3f getBackground(const Ray3f& ray) const;

	/// Sample emitter
	const Emitter *sampleEmitter(float rnd, float &pdf) const;

    float pdfEmitter(const Emitter *em) const;

	/// Get enviromental emmiter
	const Emitter *getEnvironmentalEmitter() const
	{
		return m_enviromentalEmitter;
	}

    /// Return a pointer to the scene's integrator
    const Medium* getMedium() const { return m_medium; }

    /**
     * \brief Intersect a ray against all triangles stored in the scene
     * and return detailed intersection information
     *
     * \param ray
     *    A 3-dimensional ray data structure with minimum/maximum
     *    extent information
     *
     * \param its
     *    A detailed intersection record, which will be filled by the
     *    intersection query
     *
     * \return \c true if an intersection was found
     */
    bool rayIntersect(const Ray3f &ray, Intersection &its) const {
        return m_accel->rayIntersect(ray, its, false);
    }

    /*
    * \brief Return true if there's a medium in the direction of the ray
    * If there is one add it in the .medium
    * We also will add the start and end of the medium. If there's no start of the medium the .ox will be the .o, 
    * and if there's no end, we will have .p as the .xz
    */
    bool rayIntersectMedium(const Ray3f& ray, MediumIntersection& medIts) const {

        if (!m_medium) {
            return false;
        }
        medIts.medium = m_medium;

        if (!m_medium->getBoundingBoxAsMesh()) {
            // There's no BoundingBox for the medium -> x is camera, xz is the intersection with an object
            medIts.x = medIts.o;
            medIts.xz = medIts.p;
            return true;
        }

        Intersection start_its;
        bool mediumFound = m_accel_medium->rayIntersect(ray, start_its, false);
        if (mediumFound) {
            // If we now create a ray that starts a little bit after that intersection, 
            // we can call another one to find the end:
            Ray3f end_ray(ray);
            // That 0.01 should be EPSILON_FLT
            end_ray.o = end_ray.o + 0.01 * end_ray.d;
            Intersection end_its;
            bool endMediumFound = m_accel_medium->rayIntersect(end_ray, end_its, false);
            if (endMediumFound) {
                // its has a .p that is the start of the medium:
                medIts.x = start_its.p;
                medIts.xz = end_its.p;
            }
            else {
                // If we don't find the second intersection it's because the camera is 
                // inside the bounding box, so the first intersection is the end of the medium
                medIts.x = medIts.o;
                medIts.xz = medIts.p;
            }
        }
        return mediumFound;
    }


    /**
     * \brief Intersect a ray against all triangles stored in the scene
     * and \a only determine whether or not there is an intersection.
     *
     * This method much faster than the other ray tracing function,
     * but the performance comes at the cost of not providing any
     * additional information about the detected intersection
     * (not even its position).
     *
     * \param ray
     *    A 3-dimensional ray data structure with minimum/maximum
     *    extent information
     *
     * \return \c true if an intersection was found
     */
    bool rayIntersect(const Ray3f &ray) const {
        Intersection its; /* Unused */
        return m_accel->rayIntersect(ray, its, true);
    }

    /// \brief Return an axis-aligned box that bounds the scene
    const BoundingBox3f &getBoundingBox() const {
        return m_accel->getBoundingBox();
    }

    /**
     * \brief Inherited from \ref NoriObject::activate()
     *
     * Initializes the internal data structures (kd-tree,
     * emitter sampling data structures, etc.)
     */
    void activate();

    /// Add a child object to the scene (meshes, integrators etc.)
    void addChild(NoriObject *obj, const std::string& name = "none");

    /// Return a string summary of the scene (for debugging purposes)
    std::string toString() const;

    EClassType getClassType() const { return EScene; }
private:
    std::vector<Mesh *> m_meshes;
	std::vector<Emitter *> m_emitters;
	Emitter *m_enviromentalEmitter = nullptr;
	
    Integrator *m_integrator = nullptr;
    Sampler *m_sampler = nullptr;
    Camera *m_camera = nullptr;
    Accel *m_accel = nullptr;
    Accel* m_accel_medium = nullptr;
    Medium *m_medium = nullptr;
};

NORI_NAMESPACE_END
