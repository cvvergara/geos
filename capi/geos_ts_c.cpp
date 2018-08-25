/************************************************************************
 *
 *
 * C-Wrapper for GEOS library
 *
 * Copyright (C) 2010-2012 Sandro Santilli <strk@kbt.io>
 * Copyright (C) 2005-2006 Refractions Research Inc.
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the GNU Lesser General Public Licence as published
 * by the Free Software Foundation.
 * See the COPYING file for more information.
 *
 * Author: Sandro Santilli <strk@kbt.io>
 * Thread Safety modifications: Chuck Thibert <charles.thibert@ingres.com>
 *
 ***********************************************************************/

#include <geos/platform.h>  // for FINITE
#include <geos/geom/Coordinate.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/prep/PreparedGeometry.h>
#include <geos/geom/prep/PreparedGeometryFactory.h>
#include <geos/geom/GeometryCollection.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/Point.h>
#include <geos/geom/MultiPoint.h>
#include <geos/geom/MultiLineString.h>
#include <geos/geom/MultiPolygon.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/LineSegment.h>
#include <geos/geom/LineString.h>
#include <geos/geom/PrecisionModel.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/Coordinate.h>
#include <geos/geom/IntersectionMatrix.h>
#include <geos/geom/Envelope.h>
#include <geos/index/strtree/STRtree.h>
#include <geos/index/strtree/GeometryItemDistance.h>
#include <geos/index/ItemVisitor.h>
#include <geos/io/WKTReader.h>
#include <geos/io/WKBReader.h>
#include <geos/io/WKTWriter.h>
#include <geos/io/WKBWriter.h>
#include <geos/algorithm/distance/DiscreteHausdorffDistance.h>
#include <geos/algorithm/distance/DiscreteFrechetDistance.h>
#include <geos/algorithm/CGAlgorithms.h>
#include <geos/algorithm/BoundaryNodeRule.h>
#include <geos/algorithm/MinimumDiameter.h>
#include <geos/simplify/DouglasPeuckerSimplifier.h>
#include <geos/simplify/TopologyPreservingSimplifier.h>
#include <geos/noding/GeometryNoder.h>
#include <geos/noding/Noder.h>
#include <geos/operation/buffer/BufferBuilder.h>
#include <geos/operation/buffer/BufferOp.h>
#include <geos/operation/buffer/BufferParameters.h>
#include <geos/operation/distance/DistanceOp.h>
#include <geos/operation/distance/IndexedFacetDistance.h>
#include <geos/operation/linemerge/LineMerger.h>
#include <geos/operation/overlay/OverlayOp.h>
#include <geos/operation/overlay/snap/GeometrySnapper.h>
#include <geos/operation/intersection/Rectangle.h>
#include <geos/operation/intersection/RectangleIntersection.h>
#include <geos/operation/polygonize/Polygonizer.h>
#include <geos/operation/relate/RelateOp.h>
#include <geos/operation/sharedpaths/SharedPathsOp.h>
#include <geos/operation/union/CascadedPolygonUnion.h>
#include <geos/operation/valid/IsValidOp.h>
#include <geos/precision/GeometryPrecisionReducer.h>
#include <geos/linearref/LengthIndexedLine.h>
#include <geos/triangulate/DelaunayTriangulationBuilder.h>
#include <geos/triangulate/VoronoiDiagramBuilder.h>
#include <geos/util/IllegalArgumentException.h>
#include <geos/util/Interrupt.h>
#include <geos/util/UniqueCoordinateArrayFilter.h>
#include <geos/util/Machine.h>
#include <geos/version.h>

// This should go away
#include <cmath> // finite
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <functional>

#ifdef _MSC_VER
#pragma warning(disable : 4099)
#endif

// Some extra magic to make type declarations in geos_c.h work -
// for cross-checking of types in header.
#define GEOSGeometry geos::geom::Geometry
#define GEOSPreparedGeometry geos::geom::prep::PreparedGeometry
#define GEOSCoordSequence geos::geom::CoordinateSequence
#define GEOSBufferParams geos::operation::buffer::BufferParameters
#define GEOSSTRtree geos::index::strtree::STRtree
#define GEOSWKTReader_t geos::io::WKTReader
#define GEOSWKTWriter_t geos::io::WKTWriter
#define GEOSWKBReader_t geos::io::WKBReader
#define GEOSWKBWriter_t geos::io::WKBWriter

#include "geos_c.h"
#include "../geos_revision.h"

// Intentional, to allow non-standard C elements like C99 functions to be
// imported through C++ headers of C library, like <cmath>.
using namespace std;

/// Define this if you want operations triggering Exceptions to
/// be printed.
/// (will use the NOTIFY channel - only implemented for GEOSUnion so far)
///
#undef VERBOSE_EXCEPTIONS

#include <geos/export.h>
#include <geos/precision/MinimumClearance.h>


// import the most frequently used definitions globally
using geos::geom::Geometry;
using geos::geom::LineString;
using geos::geom::LinearRing;
using geos::geom::MultiLineString;
using geos::geom::MultiPolygon;
using geos::geom::Polygon;
using geos::geom::CoordinateSequence;
using geos::geom::GeometryCollection;
using geos::geom::GeometryFactory;

using geos::io::WKTReader;
using geos::io::WKTWriter;
using geos::io::WKBReader;
using geos::io::WKBWriter;

using geos::operation::overlay::OverlayOp;
using geos::operation::overlay::overlayOp;
using geos::operation::geounion::CascadedPolygonUnion;
using geos::operation::distance::IndexedFacetDistance;
using geos::operation::buffer::BufferParameters;
using geos::operation::buffer::BufferBuilder;
using geos::precision::GeometryPrecisionReducer;
using geos::util::IllegalArgumentException;
using geos::algorithm::distance::DiscreteHausdorffDistance;
using geos::algorithm::distance::DiscreteFrechetDistance;

typedef std::unique_ptr<Geometry> GeomPtr;

typedef struct GEOSContextHandle_HS
{
    const GeometryFactory *geomFactory;
    char msgBuffer[1024];
    GEOSMessageHandler noticeMessageOld;
    GEOSMessageHandler_r noticeMessageNew;
    void *noticeData;
    GEOSMessageHandler errorMessageOld;
    GEOSMessageHandler_r errorMessageNew;
    void *errorData;
    int WKBOutputDims;
    int WKBByteOrder;
    int initialized;

    GEOSContextHandle_HS()
      :
      geomFactory(0),
      noticeMessageOld(0),
      noticeMessageNew(0),
      noticeData(0),
      errorMessageOld(0),
      errorMessageNew(0),
      errorData(0)
    {
      memset(msgBuffer, 0, sizeof(msgBuffer));
      geomFactory = GeometryFactory::getDefaultInstance();
      WKBOutputDims = 2;
      WKBByteOrder = getMachineByteOrder();
      setNoticeHandler(NULL);
      setErrorHandler(NULL);
      initialized = 1;
    }

    GEOSMessageHandler
    setNoticeHandler(GEOSMessageHandler nf)
    {
        GEOSMessageHandler f = noticeMessageOld;
        noticeMessageOld = nf;
        noticeMessageNew = NULL;
        noticeData = NULL;

        return f;
    }

    GEOSMessageHandler
    setErrorHandler(GEOSMessageHandler nf)
    {
        GEOSMessageHandler f = errorMessageOld;
        errorMessageOld = nf;
        errorMessageNew = NULL;
        errorData = NULL;

        return f;
    }

    GEOSMessageHandler_r
    setNoticeHandler(GEOSMessageHandler_r nf, void *userData) {
        GEOSMessageHandler_r f = noticeMessageNew;
        noticeMessageOld = NULL;
        noticeMessageNew = nf;
        noticeData = userData;

        return f;
    }

    GEOSMessageHandler_r
    setErrorHandler(GEOSMessageHandler_r ef, void *userData)
    {
        GEOSMessageHandler_r f = errorMessageNew;
        errorMessageOld = NULL;
        errorMessageNew = ef;
        errorData = userData;

        return f;
    }

    void
    NOTICE_MESSAGE(string fmt, ...)
    {
      if (NULL == noticeMessageOld && NULL == noticeMessageNew) {
        return;
      }

      va_list args;
      va_start(args, fmt);
      int result = vsnprintf(msgBuffer, sizeof(msgBuffer) - 1, fmt.c_str(), args);
      va_end(args);

      if (result > 0) {
        if (noticeMessageOld) {
          noticeMessageOld("%s", msgBuffer);
        } else {
          noticeMessageNew(msgBuffer, noticeData);
        }
      }
    }

    void
    ERROR_MESSAGE(string fmt, ...)
    {
      if (NULL == errorMessageOld && NULL == errorMessageNew) {
        return;
      }

      va_list args;
      va_start(args, fmt);
      int result = vsnprintf(msgBuffer, sizeof(msgBuffer) - 1, fmt.c_str(), args);
      va_end(args);

      if (result > 0) {
        if (errorMessageOld) {
          errorMessageOld("%s", msgBuffer);
        } else {
          errorMessageNew(msgBuffer, errorData);
        }
      }
    }
} GEOSContextHandleInternal_t;

// CAPI_ItemVisitor is used internally by the CAPI STRtree
// wrappers. It's defined here just to keep it out of the
// extern "C" block.
class CAPI_ItemVisitor : public geos::index::ItemVisitor {
    GEOSQueryCallback callback;
    void *userdata;
  public:
    CAPI_ItemVisitor (GEOSQueryCallback cb, void *ud)
        : ItemVisitor(), callback(cb), userdata(ud) {}
    void visitItem (void *item) override { callback(item, userdata); }
};


//## PROTOTYPES #############################################

extern "C" const char GEOS_DLL *GEOSjtsport();
extern "C" char GEOS_DLL *GEOSasText(Geometry *g1);


namespace { // anonymous

char* gstrdup_s(const char* str, const std::size_t size)
{
    char* out = static_cast<char*>(malloc(size + 1));
    if (0 != out)
    {
        // as no strlen call necessary, memcpy may be faster than strcpy
        std::memcpy(out, str, size + 1);
    }

    assert(0 != out);

    // we haven't been checking allocation before ticket #371
    if (0 == out)
    {
        throw(std::runtime_error("Failed to allocate memory for duplicate string"));
    }

    return out;
}

char* gstrdup(std::string const& str)
{
    return gstrdup_s(str.c_str(), str.size());
}

/*
 *  with lambdas that use handle
 *
 *  RT = return type
 *  HT = Handle Type
 */
template <typename RT, RT value, typename HT>
RT
execute(GEOSContextHandle_t &extHandle,
    std::function<RT(HT)> lambda)
{
    if (!extHandle) return value;
    auto handle = reinterpret_cast<HT>(extHandle);
    if (!handle->initialized) return value;

    try
    {
        return lambda(handle);
    }

    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }
    return value;
}



template <typename RT, RT value>
RT
execute(GEOSContextHandle_t &extHandle,
    std::function<RT()> lambda)
{
    if (!extHandle) return value;
    auto handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if (!handle->initialized) return value;

    try
    {
        return lambda();
    }

    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }
    return value;
}



void
execute(GEOSContextHandle_t &extHandle,
    std::function<void()> lambda)
{
    if ( 0 == extHandle )
    {
        return;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( handle->initialized == 0 )
    {
        return;
    }

    try
    {
        lambda();
    }

    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }
    return;
}




} // namespace anonymous

extern "C" {

GEOSContextHandle_t
initGEOS_r(GEOSMessageHandler nf, GEOSMessageHandler ef)
{
  GEOSContextHandle_t handle = GEOS_init_r();

  if (0 != handle) {
      GEOSContext_setNoticeHandler_r(handle, nf);
      GEOSContext_setErrorHandler_r(handle, ef);
  }

  return handle;
}

GEOSContextHandle_t
GEOS_init_r()
{
    GEOSContextHandleInternal_t *handle = new GEOSContextHandleInternal_t();

    geos::util::Interrupt::cancel();

    return static_cast<GEOSContextHandle_t>(handle);
}

GEOSMessageHandler
GEOSContext_setNoticeHandler_r(GEOSContextHandle_t extHandle, GEOSMessageHandler nf)
{
    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

    return handle->setNoticeHandler(nf);
}

GEOSMessageHandler
GEOSContext_setErrorHandler_r(GEOSContextHandle_t extHandle, GEOSMessageHandler nf)
{
    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

    return handle->setErrorHandler(nf);
}

GEOSMessageHandler_r
GEOSContext_setNoticeMessageHandler_r(GEOSContextHandle_t extHandle, GEOSMessageHandler_r nf, void *userData) {
    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

    return handle->setNoticeHandler(nf, userData);
}

GEOSMessageHandler_r
GEOSContext_setErrorMessageHandler_r(GEOSContextHandle_t extHandle, GEOSMessageHandler_r ef, void *userData)
{
    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

    return handle->setErrorHandler(ef, userData);
}

void
finishGEOS_r(GEOSContextHandle_t extHandle)
{
    // Fix up freeing handle w.r.t. malloc above
    delete extHandle;
    extHandle = NULL;
}

void
GEOS_finish_r(GEOSContextHandle_t extHandle)
{
    finishGEOS_r(extHandle);
}

void
GEOSFree_r (GEOSContextHandle_t extHandle, void* buffer)
{
    assert(0 != extHandle);

    free(buffer);
}

//-----------------------------------------------------------
// relate()-related functions
//  return 0 = false, 1 = true, 2 = error occured
//-----------------------------------------------------------
char
GEOSDisjoint_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<char, 2>(extHandle, [&](){ return g1->disjoint(g2); });
}

char
GEOSTouches_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<char, 2>(extHandle, [&](){ return g1->touches(g2);});
}

char
GEOSIntersects_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<char, 2>(extHandle, [&](){ return g1->intersects(g2);});
}

char
GEOSCrosses_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<char, 2>(extHandle, [&](){ return g1->crosses(g2); });
}

char
GEOSWithin_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<char, 2>(extHandle, [&](){ return g1->within(g2); });
}

// call g1->contains(g2)
// returns 0 = false
//         1 = true
//         2 = error was trapped
char
GEOSContains_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<char, 2>(extHandle, [&](){ return g1->contains(g2); });
}

char
GEOSOverlaps_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<char, 2>(extHandle, [&]() { return g1->overlaps(g2); });
}

char
GEOSCovers_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<char, 2>(extHandle, [&]() { return g1->covers(g2); });
}

char
GEOSCoveredBy_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<char, 2>(extHandle, [&]() { return g1->coveredBy(g2); });
}


//-------------------------------------------------------------------
// low-level relate functions
//------------------------------------------------------------------

char
GEOSRelatePattern_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2, const char *pat)
{
  return execute<char, 2>(extHandle, [&]() {
      std::string s(pat);
      return g1->relate(g2, s);
      });

}

char
GEOSRelatePatternMatch_r(GEOSContextHandle_t extHandle, const char *mat,
                           const char *pat)
{
  return execute<char, 2>(extHandle, [&]() {
        using geos::geom::IntersectionMatrix;

        std::string m(mat);
        std::string p(pat);
        IntersectionMatrix im(m);

        return im.matches(p);
    });
}

char *
GEOSRelate_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<char*, nullptr>(extHandle, [&]() -> char* {
      using geos::geom::IntersectionMatrix;

      IntersectionMatrix* im = g1->relate(g2);
      if (0 == im)
      {
      return nullptr;
      }

      char * result = gstrdup(im->toString());

      delete im;
      im = 0;

      return result;
      });
}

char *
GEOSRelateBoundaryNodeRule_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2, int bnr)
{
  return execute<char*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t * handle) -> char* {
        using geos::operation::relate::RelateOp;
        using geos::geom::IntersectionMatrix;
        using geos::algorithm::BoundaryNodeRule;

        IntersectionMatrix* im;
        switch (bnr) {
          case GEOSRELATE_BNR_MOD2: /* same as OGC */
            im = RelateOp::relate(g1, g2,
                BoundaryNodeRule::getBoundaryRuleMod2());
            break;
          case GEOSRELATE_BNR_ENDPOINT:
            im = RelateOp::relate(g1, g2,
                BoundaryNodeRule::getBoundaryEndPoint());
            break;
          case GEOSRELATE_BNR_MULTIVALENT_ENDPOINT:
            im = RelateOp::relate(g1, g2,
                BoundaryNodeRule::getBoundaryMultivalentEndPoint());
            break;
          case GEOSRELATE_BNR_MONOVALENT_ENDPOINT:
            im = RelateOp::relate(g1, g2,
                BoundaryNodeRule::getBoundaryMonovalentEndPoint());
            break;
          default:
            handle->ERROR_MESSAGE("Invalid boundary node rule %d", bnr);
            return nullptr;
          break;
        }

        if (0 == im) return nullptr;

        char *result = gstrdup(im->toString());

        delete im;
        im = nullptr;

        return result;
      });
}



//-----------------------------------------------------------------
// isValid
//-----------------------------------------------------------------


char
GEOSisValid_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<char, 2, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t * handle) {
        using geos::operation::valid::IsValidOp;
        using geos::operation::valid::TopologyValidationError;

        IsValidOp ivo(g1);
        TopologyValidationError *err = ivo.getValidationError();
        if ( err )
        {
           handle->NOTICE_MESSAGE("%s", err->toString().c_str());
           return 0;
        }
        else
        {
           return 1;
        }
    });
}

char *
GEOSisValidReason_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<char*, nullptr>(extHandle, [&]() -> char* {
        using geos::operation::valid::IsValidOp;
        using geos::operation::valid::TopologyValidationError;

        char* result = 0;
        char const* const validstr = "Valid Geometry";

        IsValidOp ivo(g1);
        TopologyValidationError *err = ivo.getValidationError();
        if (0 != err)
        {
            std::ostringstream ss;
            ss.precision(15);
            ss << err->getCoordinate();
            const std::string errloc = ss.str();
            std::string errmsg(err->getMessage());
            errmsg += "[" + errloc + "]";
            result = gstrdup(errmsg);
        }
        else
        {
            result = gstrdup(std::string(validstr));
        }

        return result;
    });
}

char
GEOSisValidDetail_r(GEOSContextHandle_t extHandle, const Geometry *g,
	int flags, char** reason, Geometry ** location)
{
  return execute<char, 2, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t * handle) {
        using geos::operation::valid::IsValidOp;
        using geos::operation::valid::TopologyValidationError;

        IsValidOp ivo(g);
        if ( flags & GEOSVALID_ALLOW_SELFTOUCHING_RING_FORMING_HOLE ) {
        	ivo.setSelfTouchingRingFormingHoleValid(true);
        }
        TopologyValidationError *err = ivo.getValidationError();
        if (0 != err)
        {
          if ( location ) {
            *location = handle->geomFactory->createPoint(err->getCoordinate());
          }
          if ( reason ) {
            std::string errmsg(err->getMessage());
            *reason = gstrdup(errmsg);
          }
          return 0;
        }

        if ( location ) *location = 0;
        if ( reason ) *reason = 0;
        return 1; /* valid */
    });
}

//-----------------------------------------------------------------
// general purpose
//-----------------------------------------------------------------

char
GEOSEquals_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<char, 2>(extHandle, [&]() { return g1->equals(g2); });
}

char
GEOSEqualsExact_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2, double tolerance)
{
  return execute<char, 2>(extHandle, [&]() {
      return g1->equalsExact(g2, tolerance);
      });
}

int
GEOSDistance_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2, double *dist)
{
  assert(0 != dist);
  return execute<int, 0>(extHandle, [&]() {
      *dist = g1->distance(g2);
      return 1;
    });
}

int
GEOSDistanceIndexed_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2, double *dist)
{
  assert(0 != dist);
  return execute<int, 0>(extHandle, [&]() {
      *dist = IndexedFacetDistance::distance(g1, g2);
      return 1;
    });
}

int
GEOSHausdorffDistance_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2, double *dist)
{
  assert(0 != dist);
  return execute<int, 0>(extHandle, [&]() {
      *dist = DiscreteHausdorffDistance::distance(*g1, *g2);
      return 1;
    });
}

int
GEOSHausdorffDistanceDensify_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2, double densifyFrac, double *dist)
{
  assert(0 != dist);
  return execute<int, 0>(extHandle, [&]() {
      *dist = DiscreteHausdorffDistance::distance(*g1, *g2, densifyFrac);
      return 1;
    });
}

int
GEOSFrechetDistance_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2, double *dist)
{
  assert(0 != dist);
  return execute<int, 0>(extHandle, [&]() {
      *dist = DiscreteFrechetDistance::distance(*g1, *g2);
      return 1;
    });
}

int
GEOSFrechetDistanceDensify_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2, double densifyFrac, double *dist)
{
  assert(0 != dist);
  return execute<int, 0>(extHandle, [&]() {
      *dist = DiscreteFrechetDistance::distance(*g1, *g2, densifyFrac);
      return 1;
    });
}

int
GEOSArea_r(GEOSContextHandle_t extHandle, const Geometry *g, double *area)
{
  assert(0 != area);
  return execute<int, 0>(extHandle, [&]() {
      *area = g->getArea();
      return 1;
    });
}

int
GEOSLength_r(GEOSContextHandle_t extHandle, const Geometry *g, double *length)
{
  assert(0 != length);
  return execute<int, 2>(extHandle, [&]() {
      *length = g->getLength();
      return 1;
    });
}

CoordinateSequence *
GEOSNearestPoints_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<CoordinateSequence*, nullptr>(extHandle, [&]() -> CoordinateSequence* {
      if (g1->isEmpty() || g2->isEmpty()) return 0;
      return geos::operation::distance::DistanceOp::nearestPoints(g1, g2);
    });
}

Geometry *
GEOSGeomFromWKT_r(GEOSContextHandle_t extHandle, const char *wkt)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        const std::string wktstring(wkt);
        WKTReader r(static_cast<GeometryFactory const*>(handle->geomFactory));

        Geometry *g = r.read(wktstring);
        return g;
    });
}

char *
GEOSGeomToWKT_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<char*, nullptr>(extHandle, [&]() -> char* {
        return gstrdup(g1->toString());
    });
}

// Remember to free the result!
unsigned char *
GEOSGeomToWKB_buf_r(GEOSContextHandle_t extHandle, const Geometry *g, size_t *size)
{
  assert(0 != size);

  return execute<unsigned char*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> unsigned char* {
        using geos::io::WKBWriter;

        int byteOrder = handle->WKBByteOrder;
        WKBWriter w(handle->WKBOutputDims, byteOrder);
        std::ostringstream os(std::ios_base::binary);
        w.write(*g, os);
        std::string wkbstring(os.str());
        const std::size_t len = wkbstring.length();

        unsigned char* result = 0;
        result = static_cast<unsigned char*>(malloc(len));
        if (0 != result)
        {
            std::memcpy(result, wkbstring.c_str(), len);
            *size = len;
        }
        return result;
    });
}

Geometry *
GEOSGeomFromWKB_buf_r(GEOSContextHandle_t extHandle, const unsigned char *wkb, size_t size)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        using geos::io::WKBReader;
        std::string wkbstring(reinterpret_cast<const char*>(wkb), size); // make it binary !
        WKBReader r(*(static_cast<GeometryFactory const*>(handle->geomFactory)));
        std::istringstream is(std::ios_base::binary);
        is.str(wkbstring);
        is.seekg(0, std::ios::beg); // rewind reader pointer
        Geometry *g = r.read(is);
        return g;
    });
}

/* Read/write wkb hex values.  Returned geometries are
   owned by the caller.*/
unsigned char *
GEOSGeomToHEX_buf_r(GEOSContextHandle_t extHandle, const Geometry *g, size_t *size)
{
  return execute<unsigned char*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> unsigned char* {
        using geos::io::WKBWriter;
        int byteOrder = handle->WKBByteOrder;
        WKBWriter w(handle->WKBOutputDims, byteOrder);
        std::ostringstream os(std::ios_base::binary);
        w.writeHEX(*g, os);
        std::string hexstring(os.str());

        char *result = gstrdup(hexstring);
        if (0 != result)
        {
            *size = hexstring.length();
        }

        return reinterpret_cast<unsigned char*>(result);
    });
}

Geometry *
GEOSGeomFromHEX_buf_r(GEOSContextHandle_t extHandle, const unsigned char *hex, size_t size)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        using geos::io::WKBReader;
        std::string hexstring(reinterpret_cast<const char*>(hex), size);
        WKBReader r(*(static_cast<GeometryFactory const*>(handle->geomFactory)));
        std::istringstream is(std::ios_base::binary);
        is.str(hexstring);
        is.seekg(0, std::ios::beg); // rewind reader pointer

        Geometry *g = r.readHEX(is);
        return g;
    });
}

char
GEOSisEmpty_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<char, 2>(extHandle, [&]() {
      return g1->isEmpty();
    });
}

char
GEOSisSimple_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<char, 2>(extHandle, [&]() {
      return g1->isSimple();
    });
}

char
GEOSisRing_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  return execute<char, 2>(extHandle, [&]() {
        const LineString *ls = dynamic_cast<const LineString *>(g);
        return ( ls ) ?  ls->isRing() : 0;
    });
}


//free the result of this
char *
GEOSGeomType_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<char*, nullptr>(extHandle, [&]() -> char* {
        std::string s = g1->getGeometryType();
        return gstrdup(s);
    });
}

// Return postgis geometry type index
int
GEOSGeomTypeId_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<int, -1>(extHandle, [&]() {
        return g1->getGeometryTypeId();
    });
}


//-------------------------------------------------------------------
// GEOS functions that return geometries
//-------------------------------------------------------------------

Geometry *
GEOSEnvelope_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
      return g1->getEnvelope();
    });
}

Geometry *
GEOSIntersection_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
      return g1->intersection(g2);
    });
}

Geometry *
GEOSBuffer_r(GEOSContextHandle_t extHandle, const Geometry *g1, double width, int quadrantsegments)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
      Geometry *g3 = g1->buffer(width, quadrantsegments);
      return g3;
    });
}

Geometry *
GEOSBufferWithStyle_r(GEOSContextHandle_t extHandle, const Geometry *g1, double width, int quadsegs, int endCapStyle, int joinStyle, double mitreLimit)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
        using geos::operation::buffer::BufferParameters;
        using geos::operation::buffer::BufferOp;
        using geos::util::IllegalArgumentException;
        BufferParameters bp;
        bp.setQuadrantSegments(quadsegs);

        if ( endCapStyle > BufferParameters::CAP_SQUARE )
        {
        	throw IllegalArgumentException("Invalid buffer endCap style");
        }
        bp.setEndCapStyle(
        	static_cast<BufferParameters::EndCapStyle>(endCapStyle)
        );

        if ( joinStyle > BufferParameters::JOIN_BEVEL )
        {
        	throw IllegalArgumentException("Invalid buffer join style");
        }
        bp.setJoinStyle(
        	static_cast<BufferParameters::JoinStyle>(joinStyle)
        );
        bp.setMitreLimit(mitreLimit);
        BufferOp op(g1, bp);
        Geometry *g3 = op.getResultGeometry(width);
        return g3;
    });
}

Geometry *
GEOSOffsetCurve_r(GEOSContextHandle_t extHandle, const Geometry *g1, double width, int quadsegs, int joinStyle, double mitreLimit)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
        BufferParameters bp;
        bp.setEndCapStyle( BufferParameters::CAP_FLAT );
        bp.setQuadrantSegments(quadsegs);

        if ( joinStyle > BufferParameters::JOIN_BEVEL )
        {
            throw IllegalArgumentException("Invalid buffer join style");
        }
        bp.setJoinStyle(
            static_cast<BufferParameters::JoinStyle>(joinStyle)
            );
        bp.setMitreLimit(mitreLimit);

        bool isLeftSide = true;
        if ( width < 0 ) {
          isLeftSide = false;
          width = -width;
        }
        BufferBuilder bufBuilder (bp);
        Geometry *g3 = bufBuilder.bufferLineSingleSided(g1, width, isLeftSide);

        return g3;
    });
}

/* @deprecated in 3.3.0 */
Geometry *
GEOSSingleSidedBuffer_r(GEOSContextHandle_t extHandle, const Geometry *g1, double width, int quadsegs, int joinStyle, double mitreLimit, int leftSide)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
        BufferParameters bp;
        bp.setEndCapStyle( BufferParameters::CAP_FLAT );
        bp.setQuadrantSegments(quadsegs);

        if ( joinStyle > BufferParameters::JOIN_BEVEL )
        {
            throw IllegalArgumentException("Invalid buffer join style");
        }
        bp.setJoinStyle(
            static_cast<BufferParameters::JoinStyle>(joinStyle)
            );
        bp.setMitreLimit(mitreLimit);

        bool isLeftSide = leftSide == 0 ? false : true;
        BufferBuilder bufBuilder (bp);
        Geometry *g3 = bufBuilder.bufferLineSingleSided(g1, width, isLeftSide);

        return g3;
    });
}

Geometry *
GEOSConvexHull_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
      return  g1->convexHull();
    });
}


Geometry *
GEOSMinimumRotatedRectangle_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
      geos::algorithm::MinimumDiameter m(g);
      return m.getMinimumRectangle();
    });
}

Geometry *
GEOSMinimumWidth_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
      geos::algorithm::MinimumDiameter m(g);
      return m.getDiameter();
    });
}

Geometry *
GEOSMinimumClearanceLine_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
      geos::precision::MinimumClearance mc(g);
      return mc.getLine().release();
    });
}

int
GEOSMinimumClearance_r(GEOSContextHandle_t extHandle, const Geometry *g, double *d)
{
  return execute<int, 2>(extHandle, [&]() {
        geos::precision::MinimumClearance mc(g);
        *d = mc.getDistance();
        return 0;
    });
}

Geometry *
GEOSDifference_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
      return g1->difference(g2);
    });
}

Geometry *
GEOSBoundary_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
      return g1->getBoundary();
    });
}

Geometry *
GEOSSymDifference_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
      return g1->symDifference(g2);
    });
}

/* TODO can not use the template because of VERBOSE_EXCEPTIONS*/
Geometry *
GEOSUnion_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *g2)
{
    if ( 0 == extHandle )
    {
        return NULL;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

    try
    {
        return g1->Union(g2);
    }
    catch (const std::exception &e)
    {
#if VERBOSE_EXCEPTIONS
        std::ostringstream s;
        s << "Exception on GEOSUnion with following inputs:" << std::endl;
        s << "A: "<<g1->toString() << std::endl;
        s << "B: "<<g2->toString() << std::endl;
        handle->NOTICE_MESSAGE("%s", s.str().c_str());
#endif // VERBOSE_EXCEPTIONS
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return NULL;
}

Geometry *
GEOSUnaryUnion_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
    if ( 0 == extHandle )
    {
        return NULL;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

    try
    {
        GeomPtr g3 ( g->Union() );
        return g3.release();
    }
    catch (const std::exception &e)
    {
#if VERBOSE_EXCEPTIONS
        std::ostringstream s;
        s << "Exception on GEOSUnaryUnion with following inputs:" << std::endl;
        s << "A: "<<g1->toString() << std::endl;
        s << "B: "<<g2->toString() << std::endl;
        handle->NOTICE_MESSAGE("%s", s.str().c_str());
#endif // VERBOSE_EXCEPTIONS
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return NULL;
}

/* TODO can not use the template because of VERBOSE_EXCEPTIONS*/
Geometry *
GEOSNode_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
    if ( 0 == extHandle )
    {
        return NULL;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

    try
    {
        std::unique_ptr<Geometry> g3 = geos::noding::GeometryNoder::node(*g);
        return g3.release();
    }
    catch (const std::exception &e)
    {
#if VERBOSE_EXCEPTIONS
        std::ostringstream s;
        s << "Exception on GEOSUnaryUnion with following inputs:" << std::endl;
        s << "A: "<<g1->toString() << std::endl;
        s << "B: "<<g2->toString() << std::endl;
        handle->NOTICE_MESSAGE("%s", s.str().c_str());
#endif // VERBOSE_EXCEPTIONS
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return NULL;
}

Geometry *
GEOSUnionCascaded_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        const geos::geom::MultiPolygon *p = dynamic_cast<const geos::geom::MultiPolygon *>(g1);
        if ( ! p )
        {
            handle->ERROR_MESSAGE("Invalid argument (must be a MultiPolygon)");
            return nullptr;
        }

        using geos::operation::geounion::CascadedPolygonUnion;
        return CascadedPolygonUnion::Union(p);
    });
}

Geometry *
GEOSPointOnSurface_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        Geometry *ret = g1->getInteriorPoint();
        if ( ! ret )
        {
            const GeometryFactory* gf = handle->geomFactory;
            // return an empty point
            return gf->createPoint();
        }
        return ret;
    });
}

/* TODO can not use the template because of VERBOSE_EXCEPTIONS*/
Geometry *
GEOSClipByRect_r(GEOSContextHandle_t extHandle, const Geometry *g, double xmin, double ymin, double xmax, double ymax)
{
    if ( 0 == extHandle )
    {
        return NULL;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

    try
    {
        using geos::operation::intersection::Rectangle;
        using geos::operation::intersection::RectangleIntersection;
        Rectangle rect(xmin, ymin, xmax, ymax);
        std::unique_ptr<Geometry> g3 = RectangleIntersection::clip(*g, rect);
        return g3.release();
    }
    catch (const std::exception &e)
    {
#if VERBOSE_EXCEPTIONS
        std::ostringstream s;
        s << "Exception on GEOSClipByRect with following inputs:" << std::endl;
        s << "A: "<<g1->toString() << std::endl;
        s << "B: "<<g2->toString() << std::endl;
        handle->NOTICE_MESSAGE("%s", s.str().c_str());
#endif // VERBOSE_EXCEPTIONS
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return NULL;
}

//-------------------------------------------------------------------
// memory management functions
//------------------------------------------------------------------

void
GEOSGeom_destroy_r(GEOSContextHandle_t extHandle, Geometry *a)
{
  execute(extHandle, [&]() -> void {
        delete a;
    });
}

void
GEOSGeom_setUserData_r(GEOSContextHandle_t extHandle, Geometry *g, void* userData)
{
  assert(0 != g);
  execute(extHandle, [&]() -> void {
      g->setUserData(userData);
    });
}

void
GEOSSetSRID_r(GEOSContextHandle_t extHandle, Geometry *g, int srid)
{
  assert(0 != g);
  execute(extHandle, [&]() -> void {
      g->setSRID(srid);
    });
}


int
GEOSGetNumCoordinates_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  assert(0 != g);
  return execute<int, -1>(extHandle, [&]() {
      return static_cast<int>(g->getNumPoints());
    });
}


/*
 * Return -1 on exception, 0 otherwise.
 * Converts Geometry to normal form (or canonical form).
 */
int
GEOSNormalize_r(GEOSContextHandle_t extHandle, Geometry *g)
{
  return execute<int, -1>(extHandle, [&]() {
        g->normalize();
        return 0; // SUCCESS
    });
}

int
GEOSGetNumInteriorRings_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<int, -1, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) {
        const Polygon *p = dynamic_cast<const Polygon *>(g1);
        if ( ! p )
        {
            handle->ERROR_MESSAGE("Argument is not a Polygon");
            return -1;
        }
        return static_cast<int>(p->getNumInteriorRing());
    });
}


// returns -1 on error and 1 for non-multi geometries
int
GEOSGetNumGeometries_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<int, -1>(extHandle, [&]() {
        return static_cast<int>(g1->getNumGeometries());
    });
}


/*
 * Call only on GEOMETRYCOLLECTION or MULTI*.
 * Return a pointer to the internal Geometry.
 */
const Geometry *
GEOSGetGeometryN_r(GEOSContextHandle_t extHandle, const Geometry *g1, int n)
{
  return execute<const Geometry*, nullptr>(extHandle, [&]() -> const Geometry* {
        return g1->getGeometryN(n);
    });
}

/*
 * Call only on LINESTRING
 * Returns NULL on exception
 */
Geometry *
GEOSGeomGetPointN_r(GEOSContextHandle_t extHandle, const Geometry *g1, int n)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        using geos::geom::LineString;
        const LineString *ls = dynamic_cast<const LineString *>(g1);
        if ( ! ls )
        {
          handle->ERROR_MESSAGE("Argument is not a LineString");
          return nullptr;
        }
        return ls->getPointN(n);
      });
}

/*
 * Call only on LINESTRING
 */
Geometry *
GEOSGeomGetStartPoint_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        using geos::geom::LineString;
        const LineString *ls = dynamic_cast<const LineString *>(g1);
        if ( ! ls )
        {
          handle->ERROR_MESSAGE("Argument is not a LineString");
          return nullptr;
        }
        return ls->getStartPoint();
    });
}

/*
 * Call only on LINESTRING
 */
Geometry *
GEOSGeomGetEndPoint_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        using geos::geom::LineString;
        const LineString *ls = dynamic_cast<const LineString *>(g1);
        if ( ! ls )
        {
          handle->ERROR_MESSAGE("Argument is not a LineString");
          return NULL;
        }
        return ls->getEndPoint();
    });
}

/*
 * Call only on LINESTRING or MULTILINESTRING
 * return 2 on exception, 1 on true, 0 on false
 */
char
GEOSisClosed_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<char, 2, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> char {
        using geos::geom::LineString;
        using geos::geom::MultiLineString;

        auto ls = dynamic_cast<const LineString *>(g1);
        if ( ls ) {
            return ls->isClosed();
        }

        auto mls = dynamic_cast<const MultiLineString *>(g1);
        if ( mls ) {
            return mls->isClosed();
        }

        handle->ERROR_MESSAGE("Argument is not a LineString or MultiLineString");
        return 2;
      });
}

/*
 * Call only on LINESTRING
 * return 0 on exception, otherwise 1
 */
int
GEOSGeomGetLength_r(GEOSContextHandle_t extHandle, const Geometry *g1, double *length)
{
  return execute<int, 0, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) {
        using geos::geom::LineString;
        const LineString *ls = dynamic_cast<const LineString *>(g1);
        if ( ! ls )
        {
          handle->ERROR_MESSAGE("Argument is not a LineString");
          return 0;
        }
        *length = ls->getLength();
        return 1;
      });
}

/*
 * Call only on LINESTRING
 */
int
GEOSGeomGetNumPoints_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<int, -1, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) {

    	  using geos::geom::LineString;
		    const LineString *ls = dynamic_cast<const LineString *>(g1);
        if ( ! ls )
        {
          handle->ERROR_MESSAGE("Argument is not a LineString");
          return -1;
        }
        return static_cast<int>(ls->getNumPoints());
    });
}

/*
 * For POINT
 * returns 0 on exception, otherwise 1
 */
int
GEOSGeomGetX_r(GEOSContextHandle_t extHandle, const Geometry *g1, double *x)
{
  return execute<int, 0, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) {
        using geos::geom::Point;
        const Point *po = dynamic_cast<const Point *>(g1);
        if ( ! po )
        {
          handle->ERROR_MESSAGE("Argument is not a Point");
          return 0;
        }
        *x = po->getX();
        return 1;
      });
}

/*
 * For POINT
 * returns 0 on exception, otherwise 1
 */
int
GEOSGeomGetY_r(GEOSContextHandle_t extHandle, const Geometry *g1, double *y)
{
  return execute<int, 0, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) {
        using geos::geom::Point;
        const Point *po = dynamic_cast<const Point *>(g1);
        if ( ! po )
        {
          handle->ERROR_MESSAGE("Argument is not a Point");
          return 0;
        }
        *y = po->getY();
        return 1;
      });
}

/*
 * For POINT
 * returns 0 on exception, otherwise 1
 */
int
GEOSGeomGetZ_r(GEOSContextHandle_t extHandle, const Geometry *g1, double *z)
{
  return execute<int, 0, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) {
        using geos::geom::Point;
        const Point *po = dynamic_cast<const Point *>(g1);
        if ( ! po )
        {
            handle->ERROR_MESSAGE("Argument is not a Point");
            return 0;
        }
        *z = po->getZ();
        return 1;
    });
}

/*
 * Call only on polygon
 * Return a copy of the internal Geometry.
 */
const Geometry *
GEOSGetExteriorRing_r(GEOSContextHandle_t extHandle, const Geometry *g1)
{
  return execute<const Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> const Geometry* {
        auto p = dynamic_cast<const Polygon *>(g1);
        if ( ! p )
        {
            handle->ERROR_MESSAGE("Invalid argument (must be a Polygon)");
            return nullptr;
        }
        return p->getExteriorRing();
    });
}

/*
 * Call only on polygon
 * Return a pointer to internal storage, do not destroy it.
 */
const Geometry *
GEOSGetInteriorRingN_r(GEOSContextHandle_t extHandle, const Geometry *g1, int n)
{
  return execute<const Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> const Geometry* {
        auto p = dynamic_cast<const Polygon *>(g1);
        if ( ! p )
        {
            handle->ERROR_MESSAGE("Invalid argument (must be a Polygon)");
            return NULL;
        }
        return p->getInteriorRingN(n);
    });
}

Geometry *
GEOSGetCentroid_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        Geometry *ret = g->getCentroid();
        if (0 == ret)
        {
            const GeometryFactory *gf = handle->geomFactory;
            return gf->createPoint();
        }
        return ret;
    });
}

/* TODO can not use the template because of GEOS_DEBUG */
Geometry *
GEOSGeom_createEmptyCollection_r(GEOSContextHandle_t extHandle, int type)
{
    if ( 0 == extHandle )
    {
        return NULL;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

#ifdef GEOS_DEBUG
    char buf[256];
    sprintf(buf, "createCollection: requested type %d", type);
    handle->NOTICE_MESSAGE("%s", buf);// TODO: Can handle->NOTICE_MESSAGE format that directly?
#endif

    try
    {
        const GeometryFactory* gf = handle->geomFactory;

        Geometry *g = 0;
        switch (type)
        {
            case GEOS_GEOMETRYCOLLECTION:
                g = gf->createGeometryCollection();
                break;
            case GEOS_MULTIPOINT:
                g = gf->createMultiPoint();
                break;
            case GEOS_MULTILINESTRING:
                g = gf->createMultiLineString();
                break;
            case GEOS_MULTIPOLYGON:
                g = gf->createMultiPolygon();
                break;
            default:
                handle->ERROR_MESSAGE("Unsupported type request for GEOSGeom_createEmptyCollection_r");
                g = 0;

        }

        return g;
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return 0;
}

/* TODO can not use the template because of GEOS_DEBUG */
Geometry *
GEOSGeom_createCollection_r(GEOSContextHandle_t extHandle, int type, Geometry **geoms, unsigned int ngeoms)
{
    if ( 0 == extHandle )
    {
        return NULL;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

#ifdef GEOS_DEBUG
    char buf[256];
    sprintf(buf, "PostGIS2GEOS_collection: requested type %d, ngeoms: %d",
            type, ngeoms);
    handle->NOTICE_MESSAGE("%s", buf);// TODO: Can handle->NOTICE_MESSAGE format that directly?
#endif

    try
    {
        const GeometryFactory* gf = handle->geomFactory;
        std::vector<Geometry*>* vgeoms = new std::vector<Geometry*>(geoms, geoms + ngeoms);

        Geometry *g = 0;
        switch (type)
        {
            case GEOS_GEOMETRYCOLLECTION:
                g = gf->createGeometryCollection(vgeoms);
                break;
            case GEOS_MULTIPOINT:
                g = gf->createMultiPoint(vgeoms);
                break;
            case GEOS_MULTILINESTRING:
                g = gf->createMultiLineString(vgeoms);
                break;
            case GEOS_MULTIPOLYGON:
                g = gf->createMultiPolygon(vgeoms);
                break;
            default:
                handle->ERROR_MESSAGE("Unsupported type request for PostGIS2GEOS_collection");
                delete vgeoms;
                g = 0;

        }

        return g;
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return 0;
}

/* TODO can not use the template because of GEOS_DEBUG */
Geometry *
GEOSPolygonize_r(GEOSContextHandle_t extHandle, const Geometry * const * g, unsigned int ngeoms)
{
    if ( 0 == extHandle )
    {
        return 0;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return 0;
    }

    Geometry *out = 0;

    try
    {
        // Polygonize
        using geos::operation::polygonize::Polygonizer;
        Polygonizer plgnzr;
        for (std::size_t i = 0; i < ngeoms; ++i)
        {
            plgnzr.add(g[i]);
        }

#if GEOS_DEBUG
        handle->NOTICE_MESSAGE("geometry vector added to polygonizer");
#endif

        auto polys = plgnzr.getPolygons();

#if GEOS_DEBUG
        handle->NOTICE_MESSAGE("output polygons got");
#endif

        // We need a vector of Geometry pointers, not Polygon pointers.
        // STL vector doesn't allow transparent upcast of this
        // nature, so we explicitly convert.
        // (it's just a waste of processor and memory, btw)
        //
        // XXX mloskot: Why not to extent GeometryFactory to accept
        // vector of polygons or extend Polygonizer to return list of Geometry*
        // or add a wrapper which semantic is similar to:
        // std::vector<as_polygon<Geometry*> >
        auto polyvec = new std::vector<Geometry *>(polys.begin(), polys.end());

        const GeometryFactory *gf = handle->geomFactory;

        // The below takes ownership of the passed vector,
        // so we must *not* delete it
        out = gf->createGeometryCollection(polyvec);
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return out;
}

/* TODO can not use the template because of GEOS_DEBUG */
Geometry *
GEOSPolygonizer_getCutEdges_r(GEOSContextHandle_t extHandle, const Geometry * const * g, unsigned int ngeoms)
{
    if ( 0 == extHandle )
    {
        return 0;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return 0;
    }

    Geometry *out = 0;

    try
    {
        // Polygonize
        using geos::operation::polygonize::Polygonizer;
        Polygonizer plgnzr;
        for (std::size_t i = 0; i < ngeoms; ++i)
        {
            plgnzr.add(g[i]);
        }

#if GEOS_DEBUG
        handle->NOTICE_MESSAGE("geometry vector added to polygonizer");
#endif

        const std::vector<const LineString *>& lines = plgnzr.getCutEdges();

#if GEOS_DEBUG
        handle->NOTICE_MESSAGE("output polygons got");
#endif

        // We need a vector of Geometry pointers, not Polygon pointers.
        // STL vector doesn't allow transparent upcast of this
        // nature, so we explicitly convert.
        // (it's just a waste of processor and memory, btw)
        // XXX mloskot: See comment for GEOSPolygonize_r
        std::vector<Geometry*> *linevec = new std::vector<Geometry *>(lines.size());

        for (std::size_t i = 0, n=lines.size(); i < n; ++i)
        {
            (*linevec)[i] = lines[i]->clone();
        }

        const GeometryFactory *gf = handle->geomFactory;

        // The below takes ownership of the passed vector,
        // so we must *not* delete it
        out = gf->createGeometryCollection(linevec);
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return out;
}

/* TODO can not use the template because of GEOS_DEBUG */
Geometry *
GEOSPolygonize_full_r(GEOSContextHandle_t extHandle, const Geometry* g,
	Geometry** cuts, Geometry** dangles, Geometry** invalid)
{
    if ( 0 == extHandle )
    {
        return 0;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return 0;
    }

    try
    {
        // Polygonize
        using geos::operation::polygonize::Polygonizer;
        Polygonizer plgnzr;
        for (std::size_t i = 0; i <g->getNumGeometries(); ++i)
        {
            plgnzr.add(g->getGeometryN(i));
        }

#if GEOS_DEBUG
        handle->NOTICE_MESSAGE("geometry vector added to polygonizer");
#endif
        const GeometryFactory *gf = handle->geomFactory;

	if ( cuts ) {

        	const std::vector<const LineString *>& lines = plgnzr.getCutEdges();
        	std::vector<Geometry*> *linevec = new std::vector<Geometry *>(lines.size());
		for (std::size_t i = 0, n=lines.size(); i < n; ++i)
		{
		    (*linevec)[i] = lines[i]->clone();
		}

		// The below takes ownership of the passed vector,
		// so we must *not* delete it
		*cuts = gf->createGeometryCollection(linevec);
	}

	if ( dangles ) {

        	const std::vector<const LineString *>& lines = plgnzr.getDangles();
        	std::vector<Geometry*> *linevec = new std::vector<Geometry *>(lines.size());
		for (std::size_t i = 0, n=lines.size(); i < n; ++i)
		{
		    (*linevec)[i] = lines[i]->clone();
		}

		// The below takes ownership of the passed vector,
		// so we must *not* delete it
		*dangles = gf->createGeometryCollection(linevec);
	}

	if ( invalid ) {

        	const std::vector<LineString *>& lines = plgnzr.getInvalidRingLines();
        	std::vector<Geometry*> *linevec = new std::vector<Geometry *>(lines.size());
		for (std::size_t i = 0, n=lines.size(); i < n; ++i)
		{
		    (*linevec)[i] = lines[i]->clone();
		}

		// The below takes ownership of the passed vector,
		// so we must *not* delete it
		*invalid = gf->createGeometryCollection(linevec);
	}

        auto polys = plgnzr.getPolygons();
        auto polyvec = new std::vector<Geometry *>(polys.begin(), polys.end());

        return gf->createGeometryCollection(polyvec);

    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
	return 0;
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
	return 0;
    }
}

/* TODO can not use the template because of GEOS_DEBUG */
Geometry *
GEOSLineMerge_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
    if ( 0 == extHandle )
    {
        return 0;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return 0;
    }

    Geometry *out = 0;

    try
    {
        using geos::operation::linemerge::LineMerger;
        LineMerger lmrgr;
        lmrgr.add(g);

        auto lines = lmrgr.getMergedLineStrings();
        assert(lines);

#if GEOS_DEBUG
        handle->NOTICE_MESSAGE("output lines got");
#endif

        std::vector<Geometry *>*geoms = new std::vector<Geometry *>(lines->size());
        for (std::vector<Geometry *>::size_type i = 0; i < lines->size(); ++i)
        {
            (*geoms)[i] = (*lines)[i];
        }
#if 0
				// is unique_ptr
        delete lines;
        lines = 0;
#endif
        const GeometryFactory *gf = handle->geomFactory;
        out = gf->buildGeometry(geoms);

        // XXX: old version
        //out = gf->createGeometryCollection(geoms);
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return out;
}

Geometry *
GEOSReverse_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
      return  g->reverse();
    });
}


void*
GEOSGeom_getUserData_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  assert(0 != g);
  return execute<void*, nullptr>(extHandle, [&]() -> void* {
      return  g->getUserData();
    });
}

int
GEOSGetSRID_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  assert(0 != g);
  return execute<int, 0>(extHandle, [&]() {
      return g->getSRID();
    });
}

const char* GEOSversion()
{
  static char version[256];
  sprintf(version, "%s " GEOS_REVISION, GEOS_CAPI_VERSION);
  return version;
}

const char* GEOSjtsport()
{
    return GEOS_JTS_PORT;
}

char
GEOSHasZ_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  assert(0 != g);

  return execute<char, -1>(extHandle, [&]() {
      assert(0 != g->getCoordinate());
      double az = g->getCoordinate()->z;
      return static_cast<char>(FINITE(az));
    });
}

int
GEOS_getWKBOutputDims_r(GEOSContextHandle_t extHandle)
{
  return execute<int, -1, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) {
        return handle->WKBOutputDims;
        });
}

int
GEOS_setWKBOutputDims_r(GEOSContextHandle_t extHandle, int newdims)
{
  return execute<int, -1, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) {
        if ( newdims < 2 || newdims > 3 )
        {
            handle->ERROR_MESSAGE("WKB output dimensions out of range 2..3");
        }

        const int olddims = handle->WKBOutputDims;
        handle->WKBOutputDims = newdims;

        return olddims;
        });
}

int
GEOS_getWKBByteOrder_r(GEOSContextHandle_t extHandle)
{
  return execute<int, -1, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) {
        return handle->WKBByteOrder;
        });
}

int
GEOS_setWKBByteOrder_r(GEOSContextHandle_t extHandle, int byteOrder)
{
  return execute<int, -1, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) {
        const int oldByteOrder = handle->WKBByteOrder;
        handle->WKBByteOrder = byteOrder;

        return oldByteOrder;
        });
}


CoordinateSequence *
GEOSCoordSeq_create_r(GEOSContextHandle_t extHandle, unsigned int size, unsigned int dims)
{
  return execute<CoordinateSequence*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> CoordinateSequence* {
        const GeometryFactory *gf = handle->geomFactory;
        return gf->getCoordinateSequenceFactory()->create(size, dims);
    });
}

int
GEOSCoordSeq_setOrdinate_r(GEOSContextHandle_t extHandle, CoordinateSequence *cs,
                           unsigned int idx, unsigned int dim, double val)
{
  assert(0 != cs);
  return execute<int, 0>(extHandle, [&]() {
        cs->setOrdinate(idx, dim, val);
        return 1;
    });
}

int
GEOSCoordSeq_setX_r(GEOSContextHandle_t extHandle, CoordinateSequence *s, unsigned int idx, double val)
{
    return GEOSCoordSeq_setOrdinate_r(extHandle, s, idx, 0, val);
}

int
GEOSCoordSeq_setY_r(GEOSContextHandle_t extHandle, CoordinateSequence *s, unsigned int idx, double val)
{
    return GEOSCoordSeq_setOrdinate_r(extHandle, s, idx, 1, val);
}

int
GEOSCoordSeq_setZ_r(GEOSContextHandle_t extHandle, CoordinateSequence *s, unsigned int idx, double val)
{
    return GEOSCoordSeq_setOrdinate_r(extHandle, s, idx, 2, val);
}

CoordinateSequence *
GEOSCoordSeq_clone_r(GEOSContextHandle_t extHandle, const CoordinateSequence *cs)
{
  assert(0 != cs);
  return execute<CoordinateSequence*, nullptr>(
      extHandle,
      [&]() -> CoordinateSequence* {
        return cs->clone();
    });
}

int
GEOSCoordSeq_getOrdinate_r(GEOSContextHandle_t extHandle, const CoordinateSequence *cs,
                           unsigned int idx, unsigned int dim, double *val)
{
  assert(0 != cs);
  assert(0 != val);

  return execute<int, 0>(extHandle, [&]() {
        *val = cs->getOrdinate(idx, dim);
        return 1;
    });
}

int
GEOSCoordSeq_getX_r(GEOSContextHandle_t extHandle, const CoordinateSequence *s, unsigned int idx, double *val)
{
    return GEOSCoordSeq_getOrdinate_r(extHandle, s, idx, 0, val);
}

int
GEOSCoordSeq_getY_r(GEOSContextHandle_t extHandle, const CoordinateSequence *s, unsigned int idx, double *val)
{
    return GEOSCoordSeq_getOrdinate_r(extHandle, s, idx, 1, val);
}

int
GEOSCoordSeq_getZ_r(GEOSContextHandle_t extHandle, const CoordinateSequence *s, unsigned int idx, double *val)
{
    return GEOSCoordSeq_getOrdinate_r(extHandle, s, idx, 2, val);
}

int
GEOSCoordSeq_getSize_r(GEOSContextHandle_t extHandle, const CoordinateSequence *cs, unsigned int *size)
{
  assert(0 != cs);
  assert(0 != size);

  return execute<int, 0>(extHandle, [&]() {
        const std::size_t sz = cs->getSize();
        *size = static_cast<unsigned int>(sz);
        return 1;
    });
}

int
GEOSCoordSeq_getDimensions_r(GEOSContextHandle_t extHandle, const CoordinateSequence *cs, unsigned int *dims)
{
  assert(0 != cs);
  assert(0 != dims);

  return execute<int, 0>(extHandle, [&]() {
      const std::size_t dim = cs->getDimension();
      *dims = static_cast<unsigned int>(dim);
      return 1;
    });
}

int
GEOSCoordSeq_isCCW_r(GEOSContextHandle_t extHandle, const CoordinateSequence *cs, char *val)
{
  assert(cs != nullptr);
  assert(val != nullptr);

  return execute<int, 0>(extHandle, [&]() {
        *val = geos::algorithm::CGAlgorithms::isCCW(cs);
        return 1;
    });
}

void
GEOSCoordSeq_destroy_r(GEOSContextHandle_t extHandle, CoordinateSequence *s)
{
  return execute(extHandle, [&]() {
        delete s;
    });
}

const CoordinateSequence *
GEOSGeom_getCoordSeq_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  return execute<const CoordinateSequence*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> const CoordinateSequence* {
        using geos::geom::Point;

        const LineString *ls = dynamic_cast<const LineString *>(g);
        if ( ls )
        {
            return ls->getCoordinatesRO();
        }

        const Point *p = dynamic_cast<const Point *>(g);
        if ( p )
        {
            return p->getCoordinatesRO();
        }

        handle->ERROR_MESSAGE("Geometry must be a Point or LineString");
        return 0;
    });
}

Geometry *
GEOSGeom_createEmptyPoint_r(GEOSContextHandle_t extHandle)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        const GeometryFactory *gf = handle->geomFactory;
        return gf->createPoint();
    });
}

Geometry *
GEOSGeom_createPoint_r(GEOSContextHandle_t extHandle, CoordinateSequence *cs)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        const GeometryFactory *gf = handle->geomFactory;
        return gf->createPoint(cs);
    });
}

Geometry *
GEOSGeom_createLinearRing_r(GEOSContextHandle_t extHandle, CoordinateSequence *cs)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        const GeometryFactory *gf = handle->geomFactory;

        return gf->createLinearRing(cs);
    });
}

Geometry *
GEOSGeom_createEmptyLineString_r(GEOSContextHandle_t extHandle)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        const GeometryFactory *gf = handle->geomFactory;

        return gf->createLineString();
    });
}

Geometry *
GEOSGeom_createLineString_r(GEOSContextHandle_t extHandle, CoordinateSequence *cs)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        const GeometryFactory *gf = handle->geomFactory;

        return gf->createLineString(cs);
    });
}

Geometry *
GEOSGeom_createEmptyPolygon_r(GEOSContextHandle_t extHandle)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        const GeometryFactory *gf = handle->geomFactory;
        return gf->createPolygon();
    });
}

Geometry *
GEOSGeom_createPolygon_r(GEOSContextHandle_t extHandle, Geometry *shell, Geometry **holes, unsigned int nholes)
{
  return execute<Geometry*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> Geometry* {
        // FIXME: holes must be non-nullptr or may be nullptr?
        //assert(0 != holes);
        using geos::geom::LinearRing;

        std::vector<Geometry *> *vholes = new std::vector<Geometry *>(holes, holes + nholes);

        LinearRing *nshell = dynamic_cast<LinearRing *>(shell);
        if ( ! nshell )
        {
            handle->ERROR_MESSAGE("Shell is not a LinearRing");
            delete vholes;
            return nullptr;
        }
        const GeometryFactory *gf = handle->geomFactory;

        return gf->createPolygon(nshell, vholes);
    });
}

Geometry *
GEOSGeom_clone_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  assert(0 != g);
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
        return g->clone();
    });
}

GEOSGeometry *
GEOSGeom_setPrecision_r(GEOSContextHandle_t extHandle, const GEOSGeometry *g,
                                          double gridSize, int flags)
{
  assert(0 != g);
  return execute<GEOSGeometry*, nullptr>(extHandle, [&]() -> GEOSGeometry* {
        using namespace geos::geom;
        const PrecisionModel *pm = g->getPrecisionModel();
        double cursize = pm->isFloating() ? 0 : 1.0/pm->getScale();
        std::unique_ptr<PrecisionModel> newpm;
        if ( gridSize ) newpm.reset( new PrecisionModel(1.0/gridSize) );
        else newpm.reset( new PrecisionModel() );
        GeometryFactory::Ptr gf =
            GeometryFactory::create( newpm.get(), g->getSRID() );
        Geometry *ret;
        if ( gridSize && cursize != gridSize )
        {
          // We need to snap the geometry
          GeometryPrecisionReducer reducer( *gf );
          reducer.setPointwise( flags & GEOS_PREC_NO_TOPO );
          reducer.setRemoveCollapsedComponents( ! (flags & GEOS_PREC_KEEP_COLLAPSED) );
          ret = reducer.reduce( *g ).release();
        }
        else
        {
          // No need or willing to snap, just change the factory
          ret = gf->createGeometry(g);
        }
        return ret;
    });
}

/* TODO can not use the template because of double */
double
GEOSGeom_getPrecision_r(GEOSContextHandle_t extHandle, const GEOSGeometry *g)
{
    using namespace geos::geom;

    assert(0 != g);

    if ( 0 == extHandle )
    {
        return -1;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return -1;
    }

    try
    {
        const PrecisionModel *pm = g->getPrecisionModel();
        double cursize = pm->isFloating() ? 0 : 1.0/pm->getScale();
        return cursize;
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return -1;
}

int
GEOSGeom_getDimensions_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  return execute<int, 0>(extHandle, [&]() {
      return (int) g->getDimension();
    });
}

int
GEOSGeom_getCoordinateDimension_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  return execute<int, 0>(extHandle, [&]() {
      return g->getCoordinateDimension();
    });
}

int
GEOSGeom_getXMin_r(GEOSContextHandle_t extHandle, const Geometry *g, double *value)
{
  return execute<int, 0>(extHandle, [&]() {
      if (g->isEmpty()) return 0;

      *value = g->getEnvelopeInternal()->getMinX();
      return 1;
    });
}

int
GEOSGeom_getXMax_r(GEOSContextHandle_t extHandle, const Geometry *g, double *value)
{
  return execute<int, 0>(extHandle, [&]() {
      if (g->isEmpty()) return 0;

      *value = g->getEnvelopeInternal()->getMaxX();
      return 1;
    });
}

int
GEOSGeom_getYMin_r(GEOSContextHandle_t extHandle, const Geometry *g, double *value)
{
  return execute<int, 0>(extHandle, [&]() {
      if (g->isEmpty()) return 0;

      *value = g->getEnvelopeInternal()->getMinY();
      return 1;
    });
}

int
GEOSGeom_getYMax_r(GEOSContextHandle_t extHandle, const Geometry *g, double *value)
{
  return execute<int, 0>(extHandle, [&]() {
      if (g->isEmpty()) return 0;

      *value = g->getEnvelopeInternal()->getMaxY();
      return 1;
    });
}

Geometry *
GEOSSimplify_r(GEOSContextHandle_t extHandle, const Geometry *g1, double tolerance)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
        using namespace geos::simplify;
        Geometry::Ptr g(DouglasPeuckerSimplifier::simplify(g1, tolerance));
        return g.release();
    });
}

Geometry *
GEOSTopologyPreserveSimplify_r(GEOSContextHandle_t extHandle, const Geometry *g1, double tolerance)
{
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
        using namespace geos::simplify;
        Geometry::Ptr g(TopologyPreservingSimplifier::simplify(g1, tolerance));
        return g.release();
    });
}

/* WKT Reader */
WKTReader *
GEOSWKTReader_create_r(GEOSContextHandle_t extHandle)
{
  return execute<WKTReader*, nullptr, GEOSContextHandleInternal_t*>(
      extHandle,
      [&](GEOSContextHandleInternal_t* handle) -> WKTReader* {
        using geos::io::WKTReader;
        return new WKTReader((GeometryFactory*)handle->geomFactory);
    });
}

void
GEOSWKTReader_destroy_r(GEOSContextHandle_t extHandle, WKTReader *reader)
{
  execute(extHandle, [&]() { delete reader; });
}

Geometry*
GEOSWKTReader_read_r(GEOSContextHandle_t extHandle, WKTReader *reader, const char *wkt)
{
  assert(0 != reader);
  return execute<Geometry*, nullptr>(extHandle, [&]() -> Geometry* {
      const std::string wktstring(wkt);
      return reader->read(wktstring);
    });
}

/* WKT Writer */
WKTWriter *
GEOSWKTWriter_create_r(GEOSContextHandle_t extHandle)
{
  return execute<WKTWriter*, nullptr>(extHandle, [&]() -> WKTWriter* {
        using geos::io::WKTWriter;
        return new WKTWriter();
    });
}

void
GEOSWKTWriter_destroy_r(GEOSContextHandle_t extHandle, WKTWriter *Writer)
{
  execute(extHandle, [&]() { delete Writer; });
}

char*
GEOSWKTWriter_write_r(GEOSContextHandle_t extHandle, WKTWriter *writer, const Geometry *geom)
{
  assert(0 != writer);

  return execute<char*, nullptr>(extHandle, [&]() -> char* {
      std::string sgeom(writer->write(geom));
      return gstrdup(sgeom);
    });
}

void
GEOSWKTWriter_setTrim_r(GEOSContextHandle_t extHandle, WKTWriter *writer, char trim)
{
  execute(extHandle, [&]() { writer->setTrim(0 != trim); });
}

void
GEOSWKTWriter_setRoundingPrecision_r(GEOSContextHandle_t extHandle, WKTWriter *writer, int precision)
{
  assert(0 != writer);
  execute(extHandle, [&]() { writer->setRoundingPrecision(precision); });
}

void
GEOSWKTWriter_setOutputDimension_r(GEOSContextHandle_t extHandle, WKTWriter *writer, int dim)
{
  assert(0 != writer);
  execute(extHandle, [&]() { writer->setOutputDimension(dim); });
}

int
GEOSWKTWriter_getOutputDimension_r(GEOSContextHandle_t extHandle, WKTWriter *writer)
{
  assert(0 != writer);
  return execute<int, -1>(extHandle, [&]() {
      return writer->getOutputDimension();
      });
}

void
GEOSWKTWriter_setOld3D_r(GEOSContextHandle_t extHandle, WKTWriter *writer, int useOld3D)
{
  assert(0 != writer);
  execute(extHandle, [&]() { writer->setOld3D(0 != useOld3D); });
}

/* WKB Reader */
WKBReader *
GEOSWKBReader_create_r(GEOSContextHandle_t extHandle)
{
    if ( 0 == extHandle )
    {
        return NULL;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

    using geos::io::WKBReader;
    try
    {
        return new WKBReader(*(GeometryFactory*)handle->geomFactory);
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return NULL;
}

void
GEOSWKBReader_destroy_r(GEOSContextHandle_t extHandle, WKBReader *reader)
{
    GEOSContextHandleInternal_t *handle = 0;

    try
    {
        delete reader;
    }
    catch (const std::exception &e)
    {
        if ( 0 == extHandle )
        {
            return;
        }

        handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
        if ( 0 == handle->initialized )
        {
            return;
        }

        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        if ( 0 == extHandle )
        {
            return;
        }

        handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
        if ( 0 == handle->initialized )
        {
            return;
        }

        handle->ERROR_MESSAGE("Unknown exception thrown");
    }
}

struct membuf : public std::streambuf
{
  membuf(char* s, std::size_t n)
  {
    setg(s, s, s + n);
  }
};

Geometry*
GEOSWKBReader_read_r(GEOSContextHandle_t extHandle, WKBReader *reader, const unsigned char *wkb, size_t size)
{
    assert(0 != reader);
    assert(0 != wkb);

    if ( 0 == extHandle )
    {
        return 0;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return 0;
    }

    try
    {
        //std::string wkbstring(reinterpret_cast<const char*>(wkb), size); // make it binary !
        //std::istringstream is(std::ios_base::binary);
        //is.str(wkbstring);
        //is.seekg(0, std::ios::beg); // rewind reader pointer

        // http://stackoverflow.com/questions/2079912/simpler-way-to-create-a-c-memorystream-from-char-size-t-without-copying-t
        membuf mb((char*)wkb, size);
        istream is(&mb);

        Geometry *g = reader->read(is);
        return g;
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return 0;
}

Geometry*
GEOSWKBReader_readHEX_r(GEOSContextHandle_t extHandle, WKBReader *reader, const unsigned char *hex, size_t size)
{
    assert(0 != reader);
    assert(0 != hex);

    if ( 0 == extHandle )
    {
        return 0;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return 0;
    }

    try
    {
        std::string hexstring(reinterpret_cast<const char*>(hex), size);
        std::istringstream is(std::ios_base::binary);
        is.str(hexstring);
        is.seekg(0, std::ios::beg); // rewind reader pointer

        Geometry *g = reader->readHEX(is);
        return g;
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return 0;
}

/* WKB Writer */
WKBWriter *
GEOSWKBWriter_create_r(GEOSContextHandle_t extHandle)
{
    if ( 0 == extHandle )
    {
        return NULL;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

    try
    {
        using geos::io::WKBWriter;
        return new WKBWriter();
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return NULL;
}

void
GEOSWKBWriter_destroy_r(GEOSContextHandle_t extHandle, WKBWriter *Writer)
{
    GEOSContextHandleInternal_t *handle = 0;

    try
    {
        delete Writer;
    }
    catch (const std::exception &e)
    {
        if ( 0 == extHandle )
        {
            return;
        }

        handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
        if ( 0 == handle->initialized )
        {
            return;
        }

        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        if ( 0 == extHandle )
        {
            return;
        }

        handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
        if ( 0 == handle->initialized )
        {
            return;
        }

        handle->ERROR_MESSAGE("Unknown exception thrown");
    }
}


/* The caller owns the result */
unsigned char*
GEOSWKBWriter_write_r(GEOSContextHandle_t extHandle, WKBWriter *writer, const Geometry *geom, size_t *size)
{
    assert(0 != writer);
    assert(0 != geom);
    assert(0 != size);

    if ( 0 == extHandle )
    {
        return NULL;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

    try
    {
        std::ostringstream os(std::ios_base::binary);
        writer->write(*geom, os);

        const std::string& wkbstring = os.str();
        const std::size_t len = wkbstring.length();

        unsigned char *result = NULL;
        result = (unsigned char*) malloc(len);
        std::memcpy(result, wkbstring.c_str(), len);
        *size = len;
        return result;
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }
    return NULL;
}

/* The caller owns the result */
unsigned char*
GEOSWKBWriter_writeHEX_r(GEOSContextHandle_t extHandle, WKBWriter *writer, const Geometry *geom, size_t *size)
{
    assert(0 != writer);
    assert(0 != geom);
    assert(0 != size);

    if ( 0 == extHandle )
    {
        return NULL;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return NULL;
    }

    try
    {
        std::ostringstream os(std::ios_base::binary);
        writer->writeHEX(*geom, os);
        std::string wkbstring(os.str());
        const std::size_t len = wkbstring.length();

        unsigned char *result = NULL;
        result = (unsigned char*) malloc(len);
        std::memcpy(result, wkbstring.c_str(), len);
        *size = len;
        return result;
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return NULL;
}

int
GEOSWKBWriter_getOutputDimension_r(GEOSContextHandle_t extHandle, const GEOSWKBWriter* writer)
{
  assert(0 != writer);

  return execute<int, 0>(extHandle, [&]() {
      return writer->getOutputDimension();
    });
}

void
GEOSWKBWriter_setOutputDimension_r(GEOSContextHandle_t extHandle, GEOSWKBWriter* writer, int newDimension)
{
    assert(0 != writer);

    if ( 0 == extHandle )
    {
        return;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 != handle->initialized )
    {
        try
        {
            writer->setOutputDimension(newDimension);
        }
        catch (const std::exception &e)
        {
            handle->ERROR_MESSAGE("%s", e.what());
        }
        catch (...)
        {
            handle->ERROR_MESSAGE("Unknown exception thrown");
        }
    }
}

int
GEOSWKBWriter_getByteOrder_r(GEOSContextHandle_t extHandle, const GEOSWKBWriter* writer)
{
    assert(0 != writer);

  return execute<int, 0>(extHandle, [&]() {
      return writer->getByteOrder();
    });
}

void
GEOSWKBWriter_setByteOrder_r(GEOSContextHandle_t extHandle, GEOSWKBWriter* writer, int newByteOrder)
{
    assert(0 != writer);

    if ( 0 == extHandle )
    {
        return;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 != handle->initialized )
    {
        try
        {
            writer->setByteOrder(newByteOrder);
        }
        catch (const std::exception &e)
        {
            handle->ERROR_MESSAGE("%s", e.what());
        }
        catch (...)
        {
            handle->ERROR_MESSAGE("Unknown exception thrown");
        }
    }
}

char
GEOSWKBWriter_getIncludeSRID_r(GEOSContextHandle_t extHandle, const GEOSWKBWriter* writer)
{
    assert(0 != writer);

  return execute<char, -1>(extHandle, [&]() {
       return static_cast<char>(writer->getIncludeSRID());
    });
}

void
GEOSWKBWriter_setIncludeSRID_r(GEOSContextHandle_t extHandle, GEOSWKBWriter* writer, const char newIncludeSRID)
{
    assert(0 != writer);

    if ( 0 == extHandle )
    {
        return;
    }

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 != handle->initialized )
    {
        try
        {
            writer->setIncludeSRID(newIncludeSRID);
        }
        catch (...)
        {
            handle->ERROR_MESSAGE("Unknown exception thrown");
        }
    }
}


//-----------------------------------------------------------------
// Prepared Geometry
//-----------------------------------------------------------------

const geos::geom::prep::PreparedGeometry*
GEOSPrepare_r(GEOSContextHandle_t extHandle, const Geometry *g)
{
  return execute<const geos::geom::prep::PreparedGeometry*, nullptr>(extHandle,
      [&]() -> const geos::geom::prep::PreparedGeometry*
      {
        return geos::geom::prep::PreparedGeometryFactory::prepare(g);
      });
}

void
GEOSPreparedGeom_destroy_r(GEOSContextHandle_t extHandle, const geos::geom::prep::PreparedGeometry *a)
{
    GEOSContextHandleInternal_t *handle = 0;

    try
    {
        delete a;
    }
    catch (const std::exception &e)
    {
        if ( 0 == extHandle )
        {
            return;
        }

        handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
        if ( 0 == handle->initialized )
        {
            return;
        }

        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        if ( 0 == extHandle )
        {
            return;
        }

        handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
        if ( 0 == handle->initialized )
        {
            return;
        }

        handle->ERROR_MESSAGE("Unknown exception thrown");
    }
}

char
GEOSPreparedContains_r(GEOSContextHandle_t extHandle,
        const geos::geom::prep::PreparedGeometry *pg, const Geometry *g)
{
  assert(0 != pg);
  assert(0 != g);

  return execute<char, 2>(extHandle, [&]() { return pg->contains(g); });
}

char
GEOSPreparedContainsProperly_r(GEOSContextHandle_t extHandle,
        const geos::geom::prep::PreparedGeometry *pg, const Geometry *g)
{
  assert(0 != pg);
  assert(0 != g);

  return execute<char, 2>(extHandle, [&]() { return pg->containsProperly(g); });
}

char
GEOSPreparedCoveredBy_r(GEOSContextHandle_t extHandle,
        const geos::geom::prep::PreparedGeometry *pg, const Geometry *g)
{
  assert(0 != pg);
  assert(0 != g);

  return execute<char, 2>(extHandle, [&]() { return pg->coveredBy(g); });
}

char
GEOSPreparedCovers_r(GEOSContextHandle_t extHandle,
        const geos::geom::prep::PreparedGeometry *pg, const Geometry *g)
{
  assert(0 != pg);
  assert(0 != g);

  return execute<char, 2>(extHandle, [&]() { return pg->covers(g); });
}

char
GEOSPreparedCrosses_r(GEOSContextHandle_t extHandle,
        const geos::geom::prep::PreparedGeometry *pg, const Geometry *g)
{
  assert(0 != pg);
  assert(0 != g);

  return execute<char, 2>(extHandle, [&]() { return pg->crosses(g); });
}

char
GEOSPreparedDisjoint_r(GEOSContextHandle_t extHandle,
        const geos::geom::prep::PreparedGeometry *pg, const Geometry *g)
{
  assert(0 != pg);
  assert(0 != g);

  return execute<char, 2>(extHandle, [&]() { return pg->disjoint(g); });
}

char
GEOSPreparedIntersects_r(GEOSContextHandle_t extHandle,
        const geos::geom::prep::PreparedGeometry *pg, const Geometry *g)
{
  assert(0 != pg);
  assert(0 != g);

  return execute<char, 2>(extHandle, [&]() { return pg->intersects(g); });
}

char
GEOSPreparedOverlaps_r(GEOSContextHandle_t extHandle,
        const geos::geom::prep::PreparedGeometry *pg, const Geometry *g)
{
  assert(0 != pg);
  assert(0 != g);

  return execute<char, 2>(extHandle, [&]() { return pg->overlaps(g); });
}

char
GEOSPreparedTouches_r(GEOSContextHandle_t extHandle,
        const geos::geom::prep::PreparedGeometry *pg, const Geometry *g)
{
  assert(0 != pg);
  assert(0 != g);

  return execute<char, 2>(extHandle, [&]() { return pg->touches(g); });
}

char
GEOSPreparedWithin_r(GEOSContextHandle_t extHandle,
        const geos::geom::prep::PreparedGeometry *pg, const Geometry *g)
{
  assert(0 != pg);
  assert(0 != g);

  return execute<char, 2>(extHandle, [&]() { return pg->within(g); });
}

//-----------------------------------------------------------------
// STRtree
//-----------------------------------------------------------------

geos::index::strtree::STRtree *
GEOSSTRtree_create_r(GEOSContextHandle_t extHandle,
                                  size_t nodeCapacity)
{
  return execute<geos::index::strtree::STRtree *, nullptr>(extHandle, [&]()
     -> geos::index::strtree::STRtree * {
        return new geos::index::strtree::STRtree(nodeCapacity);
    });
}

void
GEOSSTRtree_insert_r(GEOSContextHandle_t extHandle,
                     geos::index::strtree::STRtree *tree,
                     const geos::geom::Geometry *g,
                     void *item)
{
  assert(tree != 0);
  assert(g != 0);

  execute(extHandle, [&]() -> void {
        tree->insert(g->getEnvelopeInternal(), item);
    });
}

void
GEOSSTRtree_query_r(GEOSContextHandle_t extHandle,
                    geos::index::strtree::STRtree *tree,
                    const geos::geom::Geometry *g,
                    GEOSQueryCallback callback,
                    void *userdata)
{
  assert(tree != 0);
  assert(g != 0);
  assert(callback != 0);

  execute(extHandle, [&]() -> void {
        CAPI_ItemVisitor visitor(callback, userdata);
        tree->query(g->getEnvelopeInternal(), visitor);
        });
}

const GEOSGeometry *
GEOSSTRtree_nearest_r(GEOSContextHandle_t extHandle,
                      geos::index::strtree::STRtree *tree,
                      const geos::geom::Geometry* geom)
{
    return (const GEOSGeometry*) GEOSSTRtree_nearest_generic_r( extHandle, tree, geom, geom, nullptr, nullptr);
}

const void *
GEOSSTRtree_nearest_generic_r(GEOSContextHandle_t extHandle,
                              geos::index::strtree::STRtree *tree,
                              const void* item,
                              const geos::geom::Geometry* itemEnvelope,
                              GEOSDistanceCallback distancefn,
                              void* userdata)
{
    using namespace geos::index::strtree;

    GEOSContextHandleInternal_t *handle = 0;

		struct CustomItemDistance : public ItemDistance {
				CustomItemDistance(GEOSDistanceCallback p_distancefn, void* p_userdata)
								: m_distancefn(p_distancefn), m_userdata(p_userdata) {}

				GEOSDistanceCallback m_distancefn;
				void* m_userdata;

				double distance(const ItemBoundable* item1, const ItemBoundable* item2) override {
						const void* a = item1->getItem();
						const void* b = item2->getItem();
						double d;

						if (!m_distancefn(a, b, &d, m_userdata)) {
								throw std::runtime_error(std::string("Failed to compute distance."));
						}

						return d;
				}
		};

    try
    {
        if (distancefn) {
            CustomItemDistance itemDistance(distancefn, userdata);
            return tree->nearestNeighbour(itemEnvelope->getEnvelopeInternal(), item, &itemDistance);
        } else {
            GeometryItemDistance itemDistance = GeometryItemDistance();
            return tree->nearestNeighbour(itemEnvelope->getEnvelopeInternal(), item, &itemDistance);
        }
    }
    catch (const std::exception &e)
    {
        if ( 0 == extHandle )
        {
            return NULL;
        }

        handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
        if ( 0 == handle->initialized )
        {
            return NULL;
        }

        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        if ( 0 == extHandle )
        {
            return NULL;
        }

        handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
        if ( 0 == handle->initialized )
        {
            return NULL;
        }

        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return NULL;
}

void
GEOSSTRtree_iterate_r(GEOSContextHandle_t extHandle,
                    geos::index::strtree::STRtree *tree,
                    GEOSQueryCallback callback,
                    void *userdata)
{
  assert(tree != 0);
  assert(callback != 0);

  execute(extHandle, [&]() -> void {
      CAPI_ItemVisitor visitor(callback, userdata);
      tree->iterate(visitor);
    });
}

char
GEOSSTRtree_remove_r(GEOSContextHandle_t extHandle,
                     geos::index::strtree::STRtree *tree,
                     const geos::geom::Geometry *g,
                     void *item)
{
    assert(0 != tree);
    assert(0 != g);

    return execute<char, 2>(extHandle, [&]() -> char {
        return tree->remove(g->getEnvelopeInternal(), item);
    });
}

void
GEOSSTRtree_destroy_r(GEOSContextHandle_t extHandle,
                      geos::index::strtree::STRtree *tree)
{
  execute(extHandle, [&]() -> void {
        delete tree;
    });
}

double
GEOSProject_r(GEOSContextHandle_t extHandle,
              const Geometry *g,
              const Geometry *p)
{
    if ( 0 == extHandle ) return -1.0;
    GEOSContextHandleInternal_t *handle =
        reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( handle->initialized == 0 ) return -1.0;

    const geos::geom::Point* point = dynamic_cast<const geos::geom::Point*>(p);
    if (!point) {
        handle->ERROR_MESSAGE("third argument of GEOSProject_r must be Point*");
        return -1.0;
    }

    const geos::geom::Coordinate* inputPt = p->getCoordinate();

    try {
        return geos::linearref::LengthIndexedLine(g).project(*inputPt);
    } catch (const std::exception &e) {
        handle->ERROR_MESSAGE("%s", e.what());
        return -1.0;
    } catch (...) {
        handle->ERROR_MESSAGE("Unknown exception thrown");
        return -1.0;
    }
}


Geometry*
GEOSInterpolate_r(GEOSContextHandle_t extHandle, const Geometry *g, double d)
{
    if ( 0 == extHandle ) return 0;
    GEOSContextHandleInternal_t *handle =
        reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( handle->initialized == 0 ) return 0;

    try {
    	geos::linearref::LengthIndexedLine lil(g);
    	geos::geom::Coordinate coord = lil.extractPoint(d);
    	const GeometryFactory *gf = handle->geomFactory;
    	Geometry* point = gf->createPoint(coord);
    	return point;
    } catch (const std::exception &e) {
        handle->ERROR_MESSAGE("%s", e.what());
        return 0;
    } catch (...) {
        handle->ERROR_MESSAGE("Unknown exception thrown");
        return 0;
    }
}


double
GEOSProjectNormalized_r(GEOSContextHandle_t extHandle, const Geometry *g,
                        const Geometry *p)
{

    double length;
    GEOSLength_r(extHandle, g, &length);
    return GEOSProject_r(extHandle, g, p) / length;
}


Geometry*
GEOSInterpolateNormalized_r(GEOSContextHandle_t extHandle, const Geometry *g,
                            double d)
{
    double length;
    GEOSLength_r(extHandle, g, &length);
    return GEOSInterpolate_r(extHandle, g, d * length);
}

GEOSGeometry*
GEOSGeom_extractUniquePoints_r(GEOSContextHandle_t extHandle,
                              const GEOSGeometry* g)
{
  return execute<GEOSGeometry*, nullptr>(extHandle, [&]() -> GEOSGeometry*
    {
      using namespace geos::geom;
      using namespace geos::util;

      /* 1: extract points */
      std::vector<const Coordinate*> coords;
      UniqueCoordinateArrayFilter filter(coords);
      g->apply_ro(&filter);

      /* 2: for each point, create a geometry and put into a vector */
      std::vector<Geometry*>* points = new std::vector<Geometry*>();
      points->reserve(coords.size());
      const GeometryFactory* factory = g->getFactory();
      for (const auto c : coords)
      {
        Geometry* point = factory->createPoint(*c);
        points->push_back(point);
      }

      /* 3: create a multipoint */
      return factory->createMultiPoint(points);
    });
}

int GEOSOrientationIndex_r(GEOSContextHandle_t extHandle,
	double Ax, double Ay, double Bx, double By, double Px, double Py)
{
    GEOSContextHandleInternal_t *handle = 0;

    using geos::geom::Coordinate;
    using geos::algorithm::CGAlgorithms;

    if ( 0 == extHandle )
    {
        return 2;
    }

    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized )
    {
        return 2;
    }

    try
    {
        Coordinate A(Ax, Ay);
        Coordinate B(Bx, By);
        Coordinate P(Px, Py);
        return CGAlgorithms::orientationIndex(A, B, P);
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
        return 2;
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
        return 2;
    }
}

GEOSGeometry *
GEOSSharedPaths_r(GEOSContextHandle_t extHandle, const GEOSGeometry* g1, const GEOSGeometry* g2)
{
    using namespace geos::operation::sharedpaths;

    if ( 0 == extHandle ) return 0;
    GEOSContextHandleInternal_t *handle =
      reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( handle->initialized == 0 ) return 0;

    SharedPathsOp::PathList forw, back;
    try {
      SharedPathsOp::sharedPathsOp(*g1, *g2, forw, back);
    }
    catch (const std::exception &e)
    {
        SharedPathsOp::clearEdges(forw);
        SharedPathsOp::clearEdges(back);
        handle->ERROR_MESSAGE("%s", e.what());
        return 0;
    }
    catch (...)
    {
        SharedPathsOp::clearEdges(forw);
        SharedPathsOp::clearEdges(back);
        handle->ERROR_MESSAGE("Unknown exception thrown");
        return 0;
    }

    // Now forw and back have the geoms we want to use to construct
    // our output GeometryCollections...

    const GeometryFactory* factory = g1->getFactory();
    size_t count;

    std::unique_ptr< std::vector<Geometry*> > out1(
      new std::vector<Geometry*>()
    );
    count = forw.size();
    out1->reserve(count);
    for (size_t i=0; i<count; ++i) {
        out1->push_back(forw[i]);
    }
    std::unique_ptr<Geometry> out1g (
      factory->createMultiLineString(out1.release())
    );

    std::unique_ptr< std::vector<Geometry*> > out2(
      new std::vector<Geometry*>()
    );
    count = back.size();
    out2->reserve(count);
    for (size_t i=0; i<count; ++i) {
        out2->push_back(back[i]);
    }
    std::unique_ptr<Geometry> out2g (
      factory->createMultiLineString(out2.release())
    );

    std::unique_ptr< std::vector<Geometry*> > out(
      new std::vector<Geometry*>()
    );
    out->reserve(2);
    out->push_back(out1g.release());
    out->push_back(out2g.release());

    std::unique_ptr<Geometry> outg (
      factory->createGeometryCollection(out.release())
    );

    return outg.release();

}

GEOSGeometry *
GEOSSnap_r(GEOSContextHandle_t extHandle, const GEOSGeometry* g1,
           const GEOSGeometry* g2, double tolerance)
{
    using namespace geos::operation::overlay::snap;

    if ( 0 == extHandle ) return 0;
    GEOSContextHandleInternal_t *handle =
      reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( handle->initialized == 0 ) return 0;

    try{
      GeometrySnapper snapper( *g1 );
      std::unique_ptr<Geometry> ret = snapper.snapTo(*g2, tolerance);
      return ret.release();
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
        return 0;
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
        return 0;
    }
}

BufferParameters *
GEOSBufferParams_create_r(GEOSContextHandle_t extHandle)
{
    if ( 0 == extHandle ) return NULL;

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized ) return NULL;

    try
    {
        BufferParameters *p = new BufferParameters();
        return p;
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return 0;
}

void
GEOSBufferParams_destroy_r(GEOSContextHandle_t extHandle, BufferParameters* p)
{
  (void)extHandle;
  delete p;
}

int
GEOSBufferParams_setEndCapStyle_r(GEOSContextHandle_t extHandle,
  GEOSBufferParams* p, int style)
{
  return execute<int, 0>(extHandle, [&]()
    {
      if ( style > BufferParameters::CAP_SQUARE )
      {
        throw IllegalArgumentException("Invalid buffer endCap style");
      }
      p->setEndCapStyle(static_cast<BufferParameters::EndCapStyle>(style));
      return 1;
    });
}

int
GEOSBufferParams_setJoinStyle_r(GEOSContextHandle_t extHandle,
  GEOSBufferParams* p, int style)
{
  return execute<int, 0>(extHandle, [&]()
    {
      if ( style > BufferParameters::JOIN_BEVEL ) {
        throw IllegalArgumentException("Invalid buffer join style");
      }
      p->setJoinStyle(static_cast<BufferParameters::JoinStyle>(style));
      return 1;
    });
}

int
GEOSBufferParams_setMitreLimit_r(GEOSContextHandle_t extHandle,
  GEOSBufferParams* p, double limit)
{
  return execute<int, 0>(extHandle, [&]()
    {
        p->setMitreLimit(limit);
        return 1;
    });
}

int
GEOSBufferParams_setQuadrantSegments_r(GEOSContextHandle_t extHandle,
  GEOSBufferParams* p, int segs)
{
  return execute<int, 0>(extHandle, [&]()
    {
        p->setQuadrantSegments(segs);
        return 1;
    });
}

int
GEOSBufferParams_setSingleSided_r(GEOSContextHandle_t extHandle,
  GEOSBufferParams* p, int ss)
{
  return execute<int, 0>(extHandle, [&]()
    {
        p->setSingleSided( ss != 0 );
        return 1;
    });

}

Geometry *
GEOSBufferWithParams_r(GEOSContextHandle_t extHandle, const Geometry *g1, const BufferParameters* bp, double width)
{
    using geos::operation::buffer::BufferOp;

    if ( 0 == extHandle ) return NULL;

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized ) return NULL;

    try
    {
        BufferOp op(g1, *bp);
        Geometry *g3 = op.getResultGeometry(width);
        return g3;
    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return NULL;
}

Geometry *
GEOSDelaunayTriangulation_r(GEOSContextHandle_t extHandle, const Geometry *g1, double tolerance, int onlyEdges)
{
    if ( 0 == extHandle ) return NULL;

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized ) return NULL;

    using geos::triangulate::DelaunayTriangulationBuilder;

    try
    {
      DelaunayTriangulationBuilder builder;
      builder.setTolerance(tolerance);
      builder.setSites(*g1);

      if ( onlyEdges ) return builder.getEdges( *g1->getFactory() ).release();
      else return builder.getTriangles( *g1->getFactory() ).release();

    }
    catch (const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch (...)
    {
	    handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return NULL;
}
Geometry*
GEOSVoronoiDiagram_r(GEOSContextHandle_t extHandle, const Geometry *g1, const Geometry *env, double tolerance ,int onlyEdges)
{
	if ( 0 == extHandle ) return NULL;

	GEOSContextHandleInternal_t *handle = 0;
	handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
	if ( 0 == handle->initialized ) return NULL;

	using geos::triangulate::VoronoiDiagramBuilder;

	try
	{
		VoronoiDiagramBuilder builder;
		builder.setSites(*g1);
		builder.setTolerance(tolerance);
    if(env) builder.setClipEnvelope(env->getEnvelopeInternal());
		if(onlyEdges) return builder.getDiagramEdges(*g1->getFactory()).release();
		else return builder.getDiagram(*g1->getFactory()).release();
	}
	catch(const std::exception &e)
	{
		handle->ERROR_MESSAGE("%s", e.what());
	}
	catch(...)
	{
		handle->ERROR_MESSAGE("Unknown exception thrown");
	}

	return NULL;
}

int
GEOSSegmentIntersection_r(GEOSContextHandle_t extHandle,
    double ax0, double ay0, double ax1, double ay1,
    double bx0, double by0, double bx1, double by1,
    double* cx, double* cy)
{
    if ( 0 == extHandle ) return 0;

    GEOSContextHandleInternal_t *handle = 0;
    handle = reinterpret_cast<GEOSContextHandleInternal_t*>(extHandle);
    if ( 0 == handle->initialized ) return 0;

    try
    {
        geos::geom::LineSegment a(ax0, ay0, ax1, ay1);
        geos::geom::LineSegment b(bx0, by0, bx1, by1);
        geos::geom::Coordinate isect;

        bool intersects = a.intersection(b, isect);

        if (!intersects)
        {
            return -1;
        }

        *cx = isect.x;
        *cy = isect.y;

        return 1;
    }
    catch(const std::exception &e)
    {
        handle->ERROR_MESSAGE("%s", e.what());
    }
    catch(...)
    {
        handle->ERROR_MESSAGE("Unknown exception thrown");
    }

    return 0;
}

} /* extern "C" */

