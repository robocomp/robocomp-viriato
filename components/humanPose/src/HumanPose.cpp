// **********************************************************************
//
// Copyright (c) 2003-2017 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.7.0
//
// <auto-generated>
//
// Generated from file `HumanPose.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#include <HumanPose.h>
#include <IceUtil/PushDisableWarnings.h>
#include <Ice/LocalException.h>
#include <Ice/ValueFactory.h>
#include <Ice/OutgoingAsync.h>
#include <Ice/InputStream.h>
#include <Ice/OutputStream.h>
#include <IceUtil/PopDisableWarnings.h>

#if defined(_MSC_VER)
#   pragma warning(disable:4458) // declaration of ... hides class member
#elif defined(__clang__)
#   pragma clang diagnostic ignored "-Wshadow"
#elif defined(__GNUC__)
#   pragma GCC diagnostic ignored "-Wshadow"
#endif

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 307
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 0
#       error Ice patch level mismatch!
#   endif
#endif

#ifdef ICE_CPP11_MAPPING // C++11 mapping

namespace
{

const ::std::string iceC_RoboCompHumanPose_HumanPose_ids[2] =
{
    "::Ice::Object",
    "::RoboCompHumanPose::HumanPose"
};
const ::std::string iceC_RoboCompHumanPose_HumanPose_ops[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "obtainHumanPose"
};
const ::std::string iceC_RoboCompHumanPose_HumanPose_obtainHumanPose_name = "obtainHumanPose";

}

bool
RoboCompHumanPose::HumanPose::ice_isA(::std::string s, const ::Ice::Current&) const
{
    return ::std::binary_search(iceC_RoboCompHumanPose_HumanPose_ids, iceC_RoboCompHumanPose_HumanPose_ids + 2, s);
}

::std::vector<::std::string>
RoboCompHumanPose::HumanPose::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector<::std::string>(&iceC_RoboCompHumanPose_HumanPose_ids[0], &iceC_RoboCompHumanPose_HumanPose_ids[2]);
}

::std::string
RoboCompHumanPose::HumanPose::ice_id(const ::Ice::Current&) const
{
    return ice_staticId();
}

const ::std::string&
RoboCompHumanPose::HumanPose::ice_staticId()
{
    static const ::std::string typeId = "::RoboCompHumanPose::HumanPose";
    return typeId;
}

bool
RoboCompHumanPose::HumanPose::_iceD_obtainHumanPose(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::OperationMode::Normal, current.mode);
    auto istr = inS.startReadParams();
    ::RoboCompHumanPose::humansDetected iceP_list_of_humans;
    istr->readAll(iceP_list_of_humans);
    inS.endReadParams();
    this->obtainHumanPose(::std::move(iceP_list_of_humans), current);
    inS.writeEmptyParams();
    return true;
}

bool
RoboCompHumanPose::HumanPose::_iceDispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair<const ::std::string*, const ::std::string*> r = ::std::equal_range(iceC_RoboCompHumanPose_HumanPose_ops, iceC_RoboCompHumanPose_HumanPose_ops + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - iceC_RoboCompHumanPose_HumanPose_ops)
    {
        case 0:
        {
            return _iceD_ice_id(in, current);
        }
        case 1:
        {
            return _iceD_ice_ids(in, current);
        }
        case 2:
        {
            return _iceD_ice_isA(in, current);
        }
        case 3:
        {
            return _iceD_ice_ping(in, current);
        }
        case 4:
        {
            return _iceD_obtainHumanPose(in, current);
        }
        default:
        {
            assert(false);
            throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
        }
    }
}

void
RoboCompHumanPose::HumanPosePrx::_iceI_obtainHumanPose(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<void>>& outAsync, const ::RoboCompHumanPose::humansDetected& iceP_list_of_humans, const ::Ice::Context& context)
{
    outAsync->invoke(iceC_RoboCompHumanPose_HumanPose_obtainHumanPose_name, ::Ice::OperationMode::Normal, ::Ice::FormatType::DefaultFormat, context,
        [&](::Ice::OutputStream* ostr)
        {
            ostr->writeAll(iceP_list_of_humans);
        },
        nullptr);
}

::std::shared_ptr<::Ice::ObjectPrx>
RoboCompHumanPose::HumanPosePrx::_newInstance() const
{
    return ::IceInternal::createProxy<HumanPosePrx>();
}

const ::std::string&
RoboCompHumanPose::HumanPosePrx::ice_staticId()
{
    return RoboCompHumanPose::HumanPose::ice_staticId();
}

namespace Ice
{
}

#else // C++98 mapping

namespace
{

const ::std::string iceC_RoboCompHumanPose_HumanPose_obtainHumanPose_name = "obtainHumanPose";

}
::IceProxy::Ice::Object* ::IceProxy::RoboCompHumanPose::upCast(::IceProxy::RoboCompHumanPose::HumanPose* p) { return p; }

void
::IceProxy::RoboCompHumanPose::_readProxy(::Ice::InputStream* istr, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompHumanPose::HumanPose>& v)
{
    ::Ice::ObjectPrx proxy;
    istr->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RoboCompHumanPose::HumanPose;
        v->_copyFrom(proxy);
    }
}

::Ice::AsyncResultPtr
IceProxy::RoboCompHumanPose::HumanPose::_iceI_begin_obtainHumanPose(const ::RoboCompHumanPose::humansDetected& iceP_list_of_humans, const ::Ice::Context& context, const ::IceInternal::CallbackBasePtr& del, const ::Ice::LocalObjectPtr& cookie, bool sync)
{
    ::IceInternal::OutgoingAsyncPtr result = new ::IceInternal::CallbackOutgoing(this, iceC_RoboCompHumanPose_HumanPose_obtainHumanPose_name, del, cookie, sync);
    try
    {
        result->prepare(iceC_RoboCompHumanPose_HumanPose_obtainHumanPose_name, ::Ice::Normal, context);
        ::Ice::OutputStream* ostr = result->startWriteParams(::Ice::DefaultFormat);
        ostr->write(iceP_list_of_humans);
        result->endWriteParams();
        result->invoke(iceC_RoboCompHumanPose_HumanPose_obtainHumanPose_name);
    }
    catch(const ::Ice::Exception& ex)
    {
        result->abort(ex);
    }
    return result;
}

void
IceProxy::RoboCompHumanPose::HumanPose::end_obtainHumanPose(const ::Ice::AsyncResultPtr& result)
{
    _end(result, iceC_RoboCompHumanPose_HumanPose_obtainHumanPose_name);
}

::IceProxy::Ice::Object*
IceProxy::RoboCompHumanPose::HumanPose::_newInstance() const
{
    return new HumanPose;
}

const ::std::string&
IceProxy::RoboCompHumanPose::HumanPose::ice_staticId()
{
    return ::RoboCompHumanPose::HumanPose::ice_staticId();
}

RoboCompHumanPose::HumanPose::~HumanPose()
{
}

::Ice::Object* RoboCompHumanPose::upCast(::RoboCompHumanPose::HumanPose* p) { return p; }


namespace
{
const ::std::string iceC_RoboCompHumanPose_HumanPose_ids[2] =
{
    "::Ice::Object",
    "::RoboCompHumanPose::HumanPose"
};

}

bool
RoboCompHumanPose::HumanPose::ice_isA(const ::std::string& s, const ::Ice::Current&) const
{
    return ::std::binary_search(iceC_RoboCompHumanPose_HumanPose_ids, iceC_RoboCompHumanPose_HumanPose_ids + 2, s);
}

::std::vector< ::std::string>
RoboCompHumanPose::HumanPose::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&iceC_RoboCompHumanPose_HumanPose_ids[0], &iceC_RoboCompHumanPose_HumanPose_ids[2]);
}

const ::std::string&
RoboCompHumanPose::HumanPose::ice_id(const ::Ice::Current&) const
{
    return ice_staticId();
}

const ::std::string&
RoboCompHumanPose::HumanPose::ice_staticId()
{
#ifdef ICE_HAS_THREAD_SAFE_LOCAL_STATIC
    static const ::std::string typeId = "::RoboCompHumanPose::HumanPose";
    return typeId;
#else
    return iceC_RoboCompHumanPose_HumanPose_ids[1];
#endif
}

bool
RoboCompHumanPose::HumanPose::_iceD_obtainHumanPose(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::Normal, current.mode);
    ::Ice::InputStream* istr = inS.startReadParams();
    ::RoboCompHumanPose::humansDetected iceP_list_of_humans;
    istr->read(iceP_list_of_humans);
    inS.endReadParams();
    this->obtainHumanPose(iceP_list_of_humans, current);
    inS.writeEmptyParams();
    return true;
}

namespace
{
const ::std::string iceC_RoboCompHumanPose_HumanPose_all[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "obtainHumanPose"
};

}

bool
RoboCompHumanPose::HumanPose::_iceDispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair<const ::std::string*, const ::std::string*> r = ::std::equal_range(iceC_RoboCompHumanPose_HumanPose_all, iceC_RoboCompHumanPose_HumanPose_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - iceC_RoboCompHumanPose_HumanPose_all)
    {
        case 0:
        {
            return _iceD_ice_id(in, current);
        }
        case 1:
        {
            return _iceD_ice_ids(in, current);
        }
        case 2:
        {
            return _iceD_ice_isA(in, current);
        }
        case 3:
        {
            return _iceD_ice_ping(in, current);
        }
        case 4:
        {
            return _iceD_obtainHumanPose(in, current);
        }
        default:
        {
            assert(false);
            throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
        }
    }
}

void
RoboCompHumanPose::HumanPose::_iceWriteImpl(::Ice::OutputStream* ostr) const
{
    ostr->startSlice(ice_staticId(), -1, true);
    Ice::StreamWriter< ::RoboCompHumanPose::HumanPose, ::Ice::OutputStream>::write(ostr, *this);
    ostr->endSlice();
}

void
RoboCompHumanPose::HumanPose::_iceReadImpl(::Ice::InputStream* istr)
{
    istr->startSlice();
    Ice::StreamReader< ::RoboCompHumanPose::HumanPose, ::Ice::InputStream>::read(istr, *this);
    istr->endSlice();
}

void
RoboCompHumanPose::_icePatchObjectPtr(HumanPosePtr& handle, const ::Ice::ObjectPtr& v)
{
    handle = ::RoboCompHumanPose::HumanPosePtr::dynamicCast(v);
    if(v && !handle)
    {
        IceInternal::Ex::throwUOE(::RoboCompHumanPose::HumanPose::ice_staticId(), v);
    }
}

namespace Ice
{
}

#endif