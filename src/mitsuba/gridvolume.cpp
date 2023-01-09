/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.

    This file was edited to fit inside Nori, please go to the original 
    for a good way of doing things, don't take this one as the example
*/

#include <nori/volume.h>
#include <mitsuba/mstream.h>
#include <filesystem/resolver.h>
#include <nori/proplist.h>
#include <mitsuba/mmap.h>
#include <fstream>
#include <nori/transform.h>

#define VINTERP_NEAREST_NEIGHBOR

NORI_NAMESPACE_BEGIN

class GridDataSource : public VolumeDataSource {
public:
    enum EDataType {
        EFloat32 = 1,
        EFloat16 = 2,
        EUInt8 = 3,
        EQuantizedDirections = 4
    };

    //GridDataSource(){}

    GridDataSource(const PropertyList& propList) {
        int type_read = propList.getInteger("voltype", -1);
        //std::cout << "Voltype: " << type_read << "\n";
        if (type_read == (int)EDensity) {
            m_type = EDensity;
        }
        else if (type_read == (int)EAlbedo) {
            m_type = EAlbedo;
        }
        else if (type_read == (int)EOrientation) {
            m_type = EOrientation;
        }
        else {
            throw NoriException("Not a valid voltype set in XML!");
        }

        // _____________________________
        std::string _f_ = propList.getString("filename");
        filesystem::path resolved =
            getFileResolver()->resolve(_f_);
        m_filename = resolved;
        //std::cout << "Yay! filename gotten: " << _f_ <<"!! \n";
        // _____________________________

        m_mmap = new MemoryMappedFile(resolved);
        ref<MemoryStream> stream = new MemoryStream(m_mmap->getData(), m_mmap->getSize());
        stream->setByteOrder(Stream::ELittleEndian);
        char header[3];
        stream->read(header, 3);
        if (header[0] != 'V' || header[1] != 'O' || header[2] != 'L')
            throw NoriException("Unable to open Vol file \"%s\"!", resolved);

        uint8_t version;
        stream->read(&version, 1);
        if (version != 3)
            throw NoriException("Encountered an invalid volume data file "
                "(incorrect file version)");
        int type = stream->readInt();

        int xres = stream->readInt(),
            yres = stream->readInt(),
            zres = stream->readInt();
        m_res = Vector3i(xres, yres, zres);
        m_channels = stream->readInt();
        std::string format;

        switch (type) {
        case EFloat32:
            if (m_channels != 1 && m_channels != 3)
                throw NoriException("Encountered an unsupported float32 volume data "
                    "file (%i channels, only 1 and 3 are supported)",
                    m_channels);
            format = "float32";
            break;
        case EFloat16:
            format = "float16";
            throw NoriException("Error: float16 volumes are not yet supported!");
        case EUInt8:
            std::cout << "uint \n";
            format = "uint8";
            if (m_channels != 1 && m_channels != 3)
                throw NoriException("Encountered an unsupported uint8 volume data "
                    "file (%i channels, only 1 and 3 are supported)", m_channels);
            break;
        case EQuantizedDirections:
            std::cout << "qdir \n";
            format = "qdir";
            if (m_channels != 3)
                throw NoriException("Encountered an unsupported quantized direction "
                    "volume data file (%i channels, only 3 are supported)",
                    m_channels);
            break;
        default:
            throw NoriException("Encountered a volume data file of unknown type (type=%i, channels=%i)!", type, m_channels);
        }
        
        m_volumeType = (EDataType)type;

        if (!m_dataAABB.isValid()) {
            float xmin = stream->readSingle(),
                ymin = stream->readSingle(),
                zmin = stream->readSingle();
            float xmax = stream->readSingle(),
                ymax = stream->readSingle(),
                zmax = stream->readSingle();
            m_dataAABB = BoundingBox3f(Point3f(xmin, ymin, zmin), Point3f(xmax, ymax, zmax));
        }
        
        m_data = (uint8_t*)(((float*)m_mmap->getData()) + 12);

        /* Precompute cosine and sine lookup tables */
        /*for (int i = 0; i < 255; i++) {
            m_densityMap[i] = i / 255.0f;
        }
        m_densityMap[255] = 1.0f;*/

        /*for (int i = 0;i < 20;i++) {
            int c = 0;
            stream->read(&c,sizeof(uint8_t));
            std::cout << ";" << m_densityMap[m_data[i]] << " -> " << m_densityMap[c];
        }*/

        // Re-check the m_type based on number of channels:
        if (m_type == EDensity && m_channels != 1) {
            throw NoriException("It cannot be Density with more than 1 channel!");
        }
        else if (m_type == EOrientation && m_channels != 3) {
            throw NoriException("It cannot be Orientation with less than 3 channels!");
        }
    }

    void activate() {
        //std::cout << "\n\n________Vol activated!!! _________\n\n\n";
        Vector3f extents(m_dataAABB.getExtents());
        //m_worldToVolume = m_volumeToWorld.inverse();
        m_worldToGrid = Transform::scale(Vector3f(
            (m_res[0] - 1) / extents[0],
            (m_res[1] - 1) / extents[1],
            (m_res[2] - 1) / extents[2])
        ) * Transform::translate(-Vector3f(m_dataAABB.min)) ; //* m_worldToVolume
        m_stepSize = std::numeric_limits<Float>::infinity();
        for (int i = 0; i < 3; ++i)
            m_stepSize = std::min(m_stepSize, 0.5f * extents[i] / (Float)(m_res[i] - 1));
        m_aabb.reset();
        for (int i = 0; i < 8; ++i) {
            //m_aabb.expandBy(m_volumeToWorld(m_dataAABB.getCorner(i)));
            m_aabb.expandBy(m_dataAABB.getCorner(i));
        }

        /* Precompute cosine and sine lookup tables */
        for (int i = 0; i < 255; i++) {
            Float angle = (float)i * ((float)M_PI / 255.0f);
            m_cosPhi[i] = std::cos(2.0f * angle);
            m_sinPhi[i] = std::sin(2.0f * angle);
            m_cosTheta[i] = std::cos(angle);
            m_sinTheta[i] = std::sin(angle);
            m_densityMap[i] = i *100/ 255.0f;
        }
        m_cosPhi[255] = m_sinPhi[255] = 0;
        m_cosTheta[255] = m_sinTheta[255] = 0;
        m_densityMap[255] = 100.0f;
    }
    
    /// Look up a floating point density value by position
    Float lookupDensity(const Point3f& _p) const {

        const Point3f p = m_worldToGrid.transformAffine(_p); //HERE CHANGED THE SIGN
        const int x1 = (int) std::floor(p[0]),
            y1 = (int)std::floor(p[1]),
            z1 = (int)std::floor(p[2]),
            x2 = x1 + 1, y2 = y1 + 1, z2 = z1 + 1;
        
        if (x1 < 0 || y1 < 0 || z1 < 0 || x2 >= m_res[0] ||
            y2 >= m_res[1] || z2 >= m_res[2]) {
            std::cout << "Outside the values allowed \n";
            std::cout << "limits:" << m_res.toString() << "values: (" << x1 << ","<<y1 << "," << z1 << ")";
            return 0;
        }
        
        //std::cout << "limits:" << m_res.toString() << "values: (" << x1 << "," << y1 << "," << z1 << ")";


        const Float fx = p[0] - x1, fy = p[1] - y1, fz = p[2] - z1,
            _fx = 1.0f - fx, _fy = 1.0f - fy, _fz = 1.0f - fz;
        /*std::cout << "y: " << y1;
        std::cout << "y*r: " << (y1*m_res[0]);
        std::cout << "dens: " << m_densityMap[10];
        std::cout << "m_data[y1]: " << (int)(m_data[y1]);*/
        /*for (int i = y1;i < y1 * m_res[0] + x1;i++) {
            std::cout << "; " << m_densityMap[m_data[i]] << " -> " << i;
        }*/

        const Float
            d000 = m_densityMap[m_data[(z1 * m_res[1] + y1) * m_res[0] + x1]],
            d001 = m_densityMap[m_data[(z1 * m_res[1] + y1) * m_res[0] + x2]],
            d010 = m_densityMap[m_data[(z1 * m_res[1] + y2) * m_res[0] + x1]],
            d011 = m_densityMap[m_data[(z1 * m_res[1] + y2) * m_res[0] + x2]],
            d100 = m_densityMap[m_data[(z2 * m_res[1] + y1) * m_res[0] + x1]],
            d101 = m_densityMap[m_data[(z2 * m_res[1] + y1) * m_res[0] + x2]],
            d110 = m_densityMap[m_data[(z2 * m_res[1] + y2) * m_res[0] + x1]],
            d111 = m_densityMap[m_data[(z2 * m_res[1] + y2) * m_res[0] + x2]];
        double r = ((d000 * _fx + d001 * fx) * _fy +
            (d010 * _fx + d011 * fx) * fy) * _fz +
            ((d100 * _fx + d101 * fx) * _fy +
                (d110 * _fx + d111 * fx) * fy) * fz;
        
        /*std::cout << "sum:" << (d000 + d001 + d010 + d011 + d100 + d101 + d110 + d111);
        */
        //r = r * 100;
        return r;
        /*}
        default:
            return 0.0f;
        }*/
    };


    /// Look up a spectrum value by position
    Color3f lookupAlbedo(const Point3f& p) const {
        throw NoriException("There's no support for variable albedo!");
        return Color3f(0);
    };


    /// Look up a vector orientation by position
    Vector3f lookupOrientation(const Point3f& _p) const {
        const Point3f p = m_worldToGrid.transformAffine(_p);
        const int x1 = (int)std::floor(p[0]),
            y1 = (int)std::floor(p[1]),
            z1 = (int)std::floor(p[2]),
            x2 = x1 + 1, y2 = y1 + 1, z2 = z1 + 1;

        if (x1 < 0 || y1 < 0 || z1 < 0 || x2 >= m_res[0] ||
            y2 >= m_res[1] || z2 >= m_res[2])
            return Vector3f(0.0f);

        const Float fx = p[0] - x1, fy = p[1] - y1, fz = p[2] - z1;
        Vector3f value;

        value = lookupQuantizedDirection((((fz < .5) ? z1 : z2) * m_res[1] + ((fy < .5) ? y1 : y2)) * m_res[0] + ((fx < .5) ? x1 : x2));

        if (!value.isZero())
            return value.normalized();
        else
            return Vector3f(0.0f);

    }

    float getStepSize() const {
        return m_stepSize;
    }

    // This function shouldn't be called as a constvolume will always be a Color3f structure
    float getMaximumFloatValue() const {
        return 1.0f;
    }

    std::string toString() const {
        return "GridVolume";
    }
    
protected:

    inline Vector3f lookupQuantizedDirection(size_t index) const {
        uint8_t theta = m_data[2 * index], phi = m_data[2 * index + 1];
        return Vector3f(
            m_cosPhi[phi] * m_sinTheta[theta],
            m_sinPhi[phi] * m_sinTheta[theta],
            m_cosTheta[theta]
        );
    }

    fs::path m_filename;
    bool m_sendData;
    EDataType m_volumeType;
    
    ref<MemoryMappedFile> m_mmap;
    uint8_t* m_data;
    Vector3i m_res;

    int m_channels;
    Transform m_worldToGrid;
    //Transform m_worldToVolume;
    //Transform m_volumeToWorld;
    Float m_stepSize;
    BoundingBox3f m_dataAABB;
    
    Float m_cosTheta[256], m_sinTheta[256];
    Float m_cosPhi[256], m_sinPhi[256];
    Float m_densityMap[256];
};

NORI_REGISTER_CLASS(GridDataSource, "gridVolume");
NORI_NAMESPACE_END
