
//
// This source file is part of appleseed.
// Visit http://appleseedhq.net/ for additional information and resources.
//
// This software is released under the MIT license.
//
// Copyright (c) 2016 Esteban Tovagliari, The appleseedhq Organization
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

// Interface header.
#include "thinglassbsdf.h"

// appleseed.renderer headers.
#include "renderer/kernel/lighting/scatteringmode.h"
#include "renderer/modeling/bsdf/backfacingpolicy.h"
#include "renderer/modeling/bsdf/bsdf.h"
#include "renderer/modeling/bsdf/bsdfwrapper.h"
#include "renderer/modeling/input/inputevaluator.h"
#include "renderer/utility/paramarray.h"

// appleseed.foundation headers.
#include "foundation/math/basis.h"
#include "foundation/math/fresnel.h"
#include "foundation/math/vector.h"
#include "foundation/utility/containers/dictionary.h"
#include "foundation/utility/containers/specializedarrays.h"

// Standard headers.
#include <algorithm>
#include <cmath>
#include <string>

// Forward declarations.
namespace foundation    { class IAbortSwitch; }
namespace renderer      { class Assembly; }
namespace renderer      { class Project; }

using namespace foundation;
using namespace std;

namespace renderer
{

namespace
{
    //
    // Thin Glass BSDF.
    //
    // Simple glass shader which has no refraction.
    //

    const char* Model = "thin_glass_bsdf";

    template <typename BackfacingPolicy>
    class ThinGlassBSDFImpl
      : public BSDF
    {
      public:
        ThinGlassBSDFImpl(
            const char*         name,
            const ParamArray&   params)
          : BSDF(name, AllBSDFTypes, ScatteringMode::Specular, params)
        {
            m_inputs.declare("ior", InputFormatScalar);
        }

        virtual void release() APPLESEED_OVERRIDE
        {
            delete this;
        }

        virtual const char* get_model() const APPLESEED_OVERRIDE
        {
            return Model;
        }

        APPLESEED_FORCE_INLINE virtual void sample(
            SamplingContext&    sampling_context,
            const void*         data,
            const bool          adjoint,
            const bool          cosine_mult,
            BSDFSample&         sample) const APPLESEED_OVERRIDE
        {
            const Vector3d& shading_normal = sample.get_shading_normal();
            const double cos_on = dot(sample.m_outgoing.get_value(), shading_normal);

            const InputValues* values = static_cast<const InputValues*>(data);
            const double eta = 1.0 / values->m_ior;
            double fresnel_reflection;

            if (cos_on >= 0.0)
            {
                fresnel_reflectance_dielectric(
                    fresnel_reflection,
                    eta,
                    cos_on);
            }
            else
            {
                fresnel_reflectance_dielectric(
                    fresnel_reflection,
                    1.0 / eta,
                    -cos_on);
            }

            fresnel_reflectance_dielectric(fresnel_reflection, eta, cos_on);

            sampling_context.split_in_place(1, 1);
            const double s = sampling_context.next_double2();

            if (s < fresnel_reflection)
            {
                // reflection
                const Vector3d incoming(
                    force_above_surface(
                        reflect(sample.m_outgoing.get_value(), shading_normal),
                        sample.get_geometric_normal()));

                sample.m_value.set(static_cast<float>(fresnel_reflection / cos_in));
                sample.m_probability = DiracDelta;
                sample.m_mode = ScatteringMode::Specular;
                sample.m_incoming = Dual3d(incoming);
                sample.compute_reflected_differentials();
            }
            else
            {
                // refraction

            }



        }

        APPLESEED_FORCE_INLINE virtual double evaluate(
            const void*         data,
            const bool          adjoint,
            const bool          cosine_mult,
            const Vector3d&     geometric_normal,
            const Basis3d&      shading_basis,
            const Vector3d&     outgoing,
            const Vector3d&     incoming,
            const int           modes,
            Spectrum&           value) const APPLESEED_OVERRIDE
        {
            return 0.0;
        }

        APPLESEED_FORCE_INLINE virtual double evaluate_pdf(
            const void*         data,
            const Vector3d&     geometric_normal,
            const Basis3d&      shading_basis,
            const Vector3d&     outgoing,
            const Vector3d&     incoming,
            const int           modes) const APPLESEED_OVERRIDE
        {
            return 0.0;
        }

    private:
        typedef ThinGlassBSDFInputValues InputValues;
    };

    typedef
        BSDFWrapper<
            ThinGlassBSDFImpl<AppleseedBackfacingPolicy> > AppleseedGlassBSDF;

    typedef
        BSDFWrapper<
            ThinGlassBSDFImpl<OSLBackfacingPolicy> > OSLGlassBSDF;
}


//
// ThinGlassBSDFFactory class implementation.
//

const char* ThinGlassBSDFFactory::get_model() const
{
    return Model;
}

Dictionary ThinGlassBSDFFactory::get_model_metadata() const
{
    return
        Dictionary()
            .insert("name", Model)
            .insert("label", "Thin Glass BSDF");
}

DictionaryArray ThinGlassBSDFFactory::get_input_metadata() const
{
    DictionaryArray metadata;

    metadata.push_back(
        Dictionary()
            .insert("name", "ior")
            .insert("label", "Index of Refraction")
            .insert("type", "numeric")
            .insert("min_value", "1.0")
            .insert("max_value", "2.5")
            .insert("use", "required")
            .insert("default", "1.5"));

    return metadata;
}

auto_release_ptr<BSDF> ThinGlassBSDFFactory::create(
    const char*         name,
    const ParamArray&   params) const
{
    return auto_release_ptr<BSDF>(new AppleseedGlassBSDF(name, params));
}

auto_release_ptr<BSDF> ThinGlassBSDFFactory::create_osl(
    const char*         name,
    const ParamArray&   params) const
{
    return auto_release_ptr<BSDF>(new OSLGlassBSDF(name, params));
}

auto_release_ptr<BSDF> ThinGlassBSDFFactory::static_create(
    const char*         name,
    const ParamArray&   params)
{
    return auto_release_ptr<BSDF>(new AppleseedGlassBSDF(name, params));
}

}   // namespace renderer
