/*
*/

#include <nori/volume.h>
#include <nori/proplist.h>

NORI_NAMESPACE_BEGIN

class ConstantDataSource : public VolumeDataSource {
    public:
        ConstantDataSource(const PropertyList& props) {
            std::cout << " Albedo Aded as child to the Medium \n";
            int type = props.getInteger("voltype", -1);
            std::cout << "Voltype: " << type << "\n";
            if (type == 0) {
                m_type = EDensity;
                m_density = props.getFloat("input", 1);
                std::cout << "Added with density: " << m_density << "\n";
            }
            else if (type == 1) {
                m_type = EAlbedo;
                m_albedo = Color3f(props.getColor("input", Color3f(0.5)));
                std::cout << "Added with albedo: " << m_albedo.toString() << "\n";
            }
            else if (type == 2) {
                m_type = EOrientation;
                m_orientation = Vector3f(props.getVector("input", Vector3f(0.5)));
                std::cout << "Added with orrr: " << m_albedo.toString() << "\n";
            }
            else {
                throw NoriException("Not a valid voltype set in XML!");
            }

        }


        /// Look up a floating point density value by position
        Float lookupDensity(const Point3f& p) const {
            return m_density;
            
        };


        /// Look up a spectrum value by position
        Color3f lookupAlbedo(const Point3f& p) const {
            if (isAlbedo()) {
                return m_albedo;
            }
            return Color3f(0);
        };


        /// Look up a vector orientation by position
        Vector3f lookupOrientation(const Point3f& p) const {
            if (isOrientation()) {
                return m_orientation;
            }
            return Vector3f(0);
        }

        float getStepSize() const {
            return std::numeric_limits<float>::infinity();
        }

        // This function shouldn't be called as a constvolume will always be a Color3f structure
        float getMaximumFloatValue() const {
            return -1;
        }

        std::string toString() const {
            switch (m_type) {
            case EAlbedo:
                return tfm::format(
                    "Constvolume[\n"
                    "  color = %s\n"
                    "]", m_albedo.toString());
            case EDensity:
                return tfm::format(
                    "Constvolume[\n"
                    "  density = %f\n"
                    "]", m_density);
            case EOrientation:
                return tfm::format(
                    "Constvolume[\n"
                    "  orientation = %f\n"
                    "]", m_orientation.toString());
            default:
                break;
            }
            return tfm::format(
                "Constvolume[\n"
                "  Badly initialized!\n"
                "]");
        }
    
    protected:
        Color3f m_albedo;
        Float m_density;
        Vector3f m_orientation;
};

NORI_REGISTER_CLASS(ConstantDataSource, "constVolume");
NORI_NAMESPACE_END
