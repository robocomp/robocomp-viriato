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
// Generated from file `HumanTracker.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#include <HumanTracker.h>
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

const ::std::string iceC_RoboCompHumanTracker_HumanTracker_ids[2] =
{
    "::Ice::Object",
    "::RoboCompHumanTracker::HumanTracker"
};
const ::std::string iceC_RoboCompHumanTracker_HumanTracker_ops[] =
{
    "getJointDepthPosition",
    "getJointsPosition",
    "getRTMatrixList",
    "getUser",
    "getUserState",
    "getUsersList",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping"
};
const ::std::string iceC_RoboCompHumanTracker_HumanTracker_getJointsPosition_name = "getJointsPosition";
const ::std::string iceC_RoboCompHumanTracker_HumanTracker_getRTMatrixList_name = "getRTMatrixList";
const ::std::string iceC_RoboCompHumanTracker_HumanTracker_getUserState_name = "getUserState";
const ::std::string iceC_RoboCompHumanTracker_HumanTracker_getUser_name = "getUser";
const ::std::string iceC_RoboCompHumanTracker_HumanTracker_getUsersList_name = "getUsersList";
const ::std::string iceC_RoboCompHumanTracker_HumanTracker_getJointDepthPosition_name = "getJointDepthPosition";

}

bool
RoboCompHumanTracker::HumanTracker::ice_isA(::std::string s, const ::Ice::Current&) const
{
    return ::std::binary_search(iceC_RoboCompHumanTracker_HumanTracker_ids, iceC_RoboCompHumanTracker_HumanTracker_ids + 2, s);
}

::std::vector<::std::string>
RoboCompHumanTracker::HumanTracker::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector<::std::string>(&iceC_RoboCompHumanTracker_HumanTracker_ids[0], &iceC_RoboCompHumanTracker_HumanTracker_ids[2]);
}

::std::string
RoboCompHumanTracker::HumanTracker::ice_id(const ::Ice::Current&) const
{
    return ice_staticId();
}

const ::std::string&
RoboCompHumanTracker::HumanTracker::ice_staticId()
{
    static const ::std::string typeId = "::RoboCompHumanTracker::HumanTracker";
    return typeId;
}

bool
RoboCompHumanTracker::HumanTracker::_iceD_getJointsPosition(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::OperationMode::Normal, current.mode);
    auto istr = inS.startReadParams();
    int iceP_id;
    istr->readAll(iceP_id);
    inS.endReadParams();
    ::RoboCompHumanTracker::jointListType iceP_jointList;
    this->getJointsPosition(iceP_id, iceP_jointList, current);
    auto ostr = inS.startWriteParams();
    ostr->writeAll(iceP_jointList);
    inS.endWriteParams();
    return true;
}

bool
RoboCompHumanTracker::HumanTracker::_iceD_getRTMatrixList(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::OperationMode::Normal, current.mode);
    auto istr = inS.startReadParams();
    int iceP_id;
    istr->readAll(iceP_id);
    inS.endReadParams();
    ::RoboCompHumanTracker::RTMatrixList iceP_RTMatList;
    this->getRTMatrixList(iceP_id, iceP_RTMatList, current);
    auto ostr = inS.startWriteParams();
    ostr->writeAll(iceP_RTMatList);
    inS.endWriteParams();
    return true;
}

bool
RoboCompHumanTracker::HumanTracker::_iceD_getUserState(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::OperationMode::Normal, current.mode);
    auto istr = inS.startReadParams();
    int iceP_id;
    istr->readAll(iceP_id);
    inS.endReadParams();
    ::RoboCompHumanTracker::TrackingState iceP_state;
    this->getUserState(iceP_id, iceP_state, current);
    auto ostr = inS.startWriteParams();
    ostr->writeAll(iceP_state);
    inS.endWriteParams();
    return true;
}

bool
RoboCompHumanTracker::HumanTracker::_iceD_getUser(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::OperationMode::Normal, current.mode);
    auto istr = inS.startReadParams();
    int iceP_id;
    istr->readAll(iceP_id);
    inS.endReadParams();
    ::RoboCompHumanTracker::TPerson iceP_user;
    this->getUser(iceP_id, iceP_user, current);
    auto ostr = inS.startWriteParams();
    ostr->writeAll(iceP_user);
    inS.endWriteParams();
    return true;
}

bool
RoboCompHumanTracker::HumanTracker::_iceD_getUsersList(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::OperationMode::Normal, current.mode);
    inS.readEmptyParams();
    ::RoboCompHumanTracker::PersonList iceP_users;
    this->getUsersList(iceP_users, current);
    auto ostr = inS.startWriteParams();
    ostr->writeAll(iceP_users);
    inS.endWriteParams();
    return true;
}

bool
RoboCompHumanTracker::HumanTracker::_iceD_getJointDepthPosition(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::OperationMode::Normal, current.mode);
    auto istr = inS.startReadParams();
    int iceP_idperson;
    ::std::string iceP_idjoint;
    istr->readAll(iceP_idperson, iceP_idjoint);
    inS.endReadParams();
    ::RoboCompHumanTracker::joint iceP_depthjoint;
    bool ret = this->getJointDepthPosition(iceP_idperson, ::std::move(iceP_idjoint), iceP_depthjoint, current);
    auto ostr = inS.startWriteParams();
    ostr->writeAll(iceP_depthjoint, ret);
    inS.endWriteParams();
    return true;
}

bool
RoboCompHumanTracker::HumanTracker::_iceDispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair<const ::std::string*, const ::std::string*> r = ::std::equal_range(iceC_RoboCompHumanTracker_HumanTracker_ops, iceC_RoboCompHumanTracker_HumanTracker_ops + 10, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - iceC_RoboCompHumanTracker_HumanTracker_ops)
    {
        case 0:
        {
            return _iceD_getJointDepthPosition(in, current);
        }
        case 1:
        {
            return _iceD_getJointsPosition(in, current);
        }
        case 2:
        {
            return _iceD_getRTMatrixList(in, current);
        }
        case 3:
        {
            return _iceD_getUser(in, current);
        }
        case 4:
        {
            return _iceD_getUserState(in, current);
        }
        case 5:
        {
            return _iceD_getUsersList(in, current);
        }
        case 6:
        {
            return _iceD_ice_id(in, current);
        }
        case 7:
        {
            return _iceD_ice_ids(in, current);
        }
        case 8:
        {
            return _iceD_ice_isA(in, current);
        }
        case 9:
        {
            return _iceD_ice_ping(in, current);
        }
        default:
        {
            assert(false);
            throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
        }
    }
}

void
RoboCompHumanTracker::HumanTrackerPrx::_iceI_getJointsPosition(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompHumanTracker::jointListType>>& outAsync, int iceP_id, const ::Ice::Context& context)
{
    _checkTwowayOnly(iceC_RoboCompHumanTracker_HumanTracker_getJointsPosition_name);
    outAsync->invoke(iceC_RoboCompHumanTracker_HumanTracker_getJointsPosition_name, ::Ice::OperationMode::Normal, ::Ice::FormatType::DefaultFormat, context,
        [&](::Ice::OutputStream* ostr)
        {
            ostr->writeAll(iceP_id);
        },
        nullptr);
}

void
RoboCompHumanTracker::HumanTrackerPrx::_iceI_getRTMatrixList(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompHumanTracker::RTMatrixList>>& outAsync, int iceP_id, const ::Ice::Context& context)
{
    _checkTwowayOnly(iceC_RoboCompHumanTracker_HumanTracker_getRTMatrixList_name);
    outAsync->invoke(iceC_RoboCompHumanTracker_HumanTracker_getRTMatrixList_name, ::Ice::OperationMode::Normal, ::Ice::FormatType::DefaultFormat, context,
        [&](::Ice::OutputStream* ostr)
        {
            ostr->writeAll(iceP_id);
        },
        nullptr);
}

void
RoboCompHumanTracker::HumanTrackerPrx::_iceI_getUserState(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompHumanTracker::TrackingState>>& outAsync, int iceP_id, const ::Ice::Context& context)
{
    _checkTwowayOnly(iceC_RoboCompHumanTracker_HumanTracker_getUserState_name);
    outAsync->invoke(iceC_RoboCompHumanTracker_HumanTracker_getUserState_name, ::Ice::OperationMode::Normal, ::Ice::FormatType::DefaultFormat, context,
        [&](::Ice::OutputStream* ostr)
        {
            ostr->writeAll(iceP_id);
        },
        nullptr);
}

void
RoboCompHumanTracker::HumanTrackerPrx::_iceI_getUser(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompHumanTracker::TPerson>>& outAsync, int iceP_id, const ::Ice::Context& context)
{
    _checkTwowayOnly(iceC_RoboCompHumanTracker_HumanTracker_getUser_name);
    outAsync->invoke(iceC_RoboCompHumanTracker_HumanTracker_getUser_name, ::Ice::OperationMode::Normal, ::Ice::FormatType::DefaultFormat, context,
        [&](::Ice::OutputStream* ostr)
        {
            ostr->writeAll(iceP_id);
        },
        nullptr);
}

void
RoboCompHumanTracker::HumanTrackerPrx::_iceI_getUsersList(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompHumanTracker::PersonList>>& outAsync, const ::Ice::Context& context)
{
    _checkTwowayOnly(iceC_RoboCompHumanTracker_HumanTracker_getUsersList_name);
    outAsync->invoke(iceC_RoboCompHumanTracker_HumanTracker_getUsersList_name, ::Ice::OperationMode::Normal, ::Ice::FormatType::DefaultFormat, context,
        nullptr,
        nullptr);
}

void
RoboCompHumanTracker::HumanTrackerPrx::_iceI_getJointDepthPosition(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompHumanTracker::HumanTracker::GetJointDepthPositionResult>>& outAsync, int iceP_idperson, const ::std::string& iceP_idjoint, const ::Ice::Context& context)
{
    _checkTwowayOnly(iceC_RoboCompHumanTracker_HumanTracker_getJointDepthPosition_name);
    outAsync->invoke(iceC_RoboCompHumanTracker_HumanTracker_getJointDepthPosition_name, ::Ice::OperationMode::Normal, ::Ice::FormatType::DefaultFormat, context,
        [&](::Ice::OutputStream* ostr)
        {
            ostr->writeAll(iceP_idperson, iceP_idjoint);
        },
        nullptr,
        [](::Ice::InputStream* istr)
        {
            ::RoboCompHumanTracker::HumanTracker::GetJointDepthPositionResult v;
            istr->readAll(v.depthjoint, v.returnValue);
            return v;
        });
}

::std::shared_ptr<::Ice::ObjectPrx>
RoboCompHumanTracker::HumanTrackerPrx::_newInstance() const
{
    return ::IceInternal::createProxy<HumanTrackerPrx>();
}

const ::std::string&
RoboCompHumanTracker::HumanTrackerPrx::ice_staticId()
{
    return RoboCompHumanTracker::HumanTracker::ice_staticId();
}

namespace Ice
{
}

#else // C++98 mapping

namespace
{

const ::std::string iceC_RoboCompHumanTracker_HumanTracker_getJointsPosition_name = "getJointsPosition";

const ::std::string iceC_RoboCompHumanTracker_HumanTracker_getRTMatrixList_name = "getRTMatrixList";

const ::std::string iceC_RoboCompHumanTracker_HumanTracker_getUserState_name = "getUserState";

const ::std::string iceC_RoboCompHumanTracker_HumanTracker_getUser_name = "getUser";

const ::std::string iceC_RoboCompHumanTracker_HumanTracker_getUsersList_name = "getUsersList";

const ::std::string iceC_RoboCompHumanTracker_HumanTracker_getJointDepthPosition_name = "getJointDepthPosition";

}
::IceProxy::Ice::Object* ::IceProxy::RoboCompHumanTracker::upCast(::IceProxy::RoboCompHumanTracker::HumanTracker* p) { return p; }

void
::IceProxy::RoboCompHumanTracker::_readProxy(::Ice::InputStream* istr, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompHumanTracker::HumanTracker>& v)
{
    ::Ice::ObjectPrx proxy;
    istr->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RoboCompHumanTracker::HumanTracker;
        v->_copyFrom(proxy);
    }
}

::Ice::AsyncResultPtr
IceProxy::RoboCompHumanTracker::HumanTracker::_iceI_begin_getJointsPosition(::Ice::Int iceP_id, const ::Ice::Context& context, const ::IceInternal::CallbackBasePtr& del, const ::Ice::LocalObjectPtr& cookie, bool sync)
{
    _checkTwowayOnly(iceC_RoboCompHumanTracker_HumanTracker_getJointsPosition_name, sync);
    ::IceInternal::OutgoingAsyncPtr result = new ::IceInternal::CallbackOutgoing(this, iceC_RoboCompHumanTracker_HumanTracker_getJointsPosition_name, del, cookie, sync);
    try
    {
        result->prepare(iceC_RoboCompHumanTracker_HumanTracker_getJointsPosition_name, ::Ice::Normal, context);
        ::Ice::OutputStream* ostr = result->startWriteParams(::Ice::DefaultFormat);
        ostr->write(iceP_id);
        result->endWriteParams();
        result->invoke(iceC_RoboCompHumanTracker_HumanTracker_getJointsPosition_name);
    }
    catch(const ::Ice::Exception& ex)
    {
        result->abort(ex);
    }
    return result;
}

void
IceProxy::RoboCompHumanTracker::HumanTracker::end_getJointsPosition(::RoboCompHumanTracker::jointListType& iceP_jointList, const ::Ice::AsyncResultPtr& result)
{
    ::Ice::AsyncResult::_check(result, this, iceC_RoboCompHumanTracker_HumanTracker_getJointsPosition_name);
    if(!result->_waitForResponse())
    {
        try
        {
            result->_throwUserException();
        }
        catch(const ::Ice::UserException& ex)
        {
            throw ::Ice::UnknownUserException(__FILE__, __LINE__, ex.ice_id());
        }
    }
    ::Ice::InputStream* istr = result->_startReadParams();
    istr->read(iceP_jointList);
    result->_endReadParams();
}

::Ice::AsyncResultPtr
IceProxy::RoboCompHumanTracker::HumanTracker::_iceI_begin_getRTMatrixList(::Ice::Int iceP_id, const ::Ice::Context& context, const ::IceInternal::CallbackBasePtr& del, const ::Ice::LocalObjectPtr& cookie, bool sync)
{
    _checkTwowayOnly(iceC_RoboCompHumanTracker_HumanTracker_getRTMatrixList_name, sync);
    ::IceInternal::OutgoingAsyncPtr result = new ::IceInternal::CallbackOutgoing(this, iceC_RoboCompHumanTracker_HumanTracker_getRTMatrixList_name, del, cookie, sync);
    try
    {
        result->prepare(iceC_RoboCompHumanTracker_HumanTracker_getRTMatrixList_name, ::Ice::Normal, context);
        ::Ice::OutputStream* ostr = result->startWriteParams(::Ice::DefaultFormat);
        ostr->write(iceP_id);
        result->endWriteParams();
        result->invoke(iceC_RoboCompHumanTracker_HumanTracker_getRTMatrixList_name);
    }
    catch(const ::Ice::Exception& ex)
    {
        result->abort(ex);
    }
    return result;
}

void
IceProxy::RoboCompHumanTracker::HumanTracker::end_getRTMatrixList(::RoboCompHumanTracker::RTMatrixList& iceP_RTMatList, const ::Ice::AsyncResultPtr& result)
{
    ::Ice::AsyncResult::_check(result, this, iceC_RoboCompHumanTracker_HumanTracker_getRTMatrixList_name);
    if(!result->_waitForResponse())
    {
        try
        {
            result->_throwUserException();
        }
        catch(const ::Ice::UserException& ex)
        {
            throw ::Ice::UnknownUserException(__FILE__, __LINE__, ex.ice_id());
        }
    }
    ::Ice::InputStream* istr = result->_startReadParams();
    istr->read(iceP_RTMatList);
    result->_endReadParams();
}

::Ice::AsyncResultPtr
IceProxy::RoboCompHumanTracker::HumanTracker::_iceI_begin_getUserState(::Ice::Int iceP_id, const ::Ice::Context& context, const ::IceInternal::CallbackBasePtr& del, const ::Ice::LocalObjectPtr& cookie, bool sync)
{
    _checkTwowayOnly(iceC_RoboCompHumanTracker_HumanTracker_getUserState_name, sync);
    ::IceInternal::OutgoingAsyncPtr result = new ::IceInternal::CallbackOutgoing(this, iceC_RoboCompHumanTracker_HumanTracker_getUserState_name, del, cookie, sync);
    try
    {
        result->prepare(iceC_RoboCompHumanTracker_HumanTracker_getUserState_name, ::Ice::Normal, context);
        ::Ice::OutputStream* ostr = result->startWriteParams(::Ice::DefaultFormat);
        ostr->write(iceP_id);
        result->endWriteParams();
        result->invoke(iceC_RoboCompHumanTracker_HumanTracker_getUserState_name);
    }
    catch(const ::Ice::Exception& ex)
    {
        result->abort(ex);
    }
    return result;
}

void
IceProxy::RoboCompHumanTracker::HumanTracker::end_getUserState(::RoboCompHumanTracker::TrackingState& iceP_state, const ::Ice::AsyncResultPtr& result)
{
    ::Ice::AsyncResult::_check(result, this, iceC_RoboCompHumanTracker_HumanTracker_getUserState_name);
    if(!result->_waitForResponse())
    {
        try
        {
            result->_throwUserException();
        }
        catch(const ::Ice::UserException& ex)
        {
            throw ::Ice::UnknownUserException(__FILE__, __LINE__, ex.ice_id());
        }
    }
    ::Ice::InputStream* istr = result->_startReadParams();
    istr->read(iceP_state);
    result->_endReadParams();
}

::Ice::AsyncResultPtr
IceProxy::RoboCompHumanTracker::HumanTracker::_iceI_begin_getUser(::Ice::Int iceP_id, const ::Ice::Context& context, const ::IceInternal::CallbackBasePtr& del, const ::Ice::LocalObjectPtr& cookie, bool sync)
{
    _checkTwowayOnly(iceC_RoboCompHumanTracker_HumanTracker_getUser_name, sync);
    ::IceInternal::OutgoingAsyncPtr result = new ::IceInternal::CallbackOutgoing(this, iceC_RoboCompHumanTracker_HumanTracker_getUser_name, del, cookie, sync);
    try
    {
        result->prepare(iceC_RoboCompHumanTracker_HumanTracker_getUser_name, ::Ice::Normal, context);
        ::Ice::OutputStream* ostr = result->startWriteParams(::Ice::DefaultFormat);
        ostr->write(iceP_id);
        result->endWriteParams();
        result->invoke(iceC_RoboCompHumanTracker_HumanTracker_getUser_name);
    }
    catch(const ::Ice::Exception& ex)
    {
        result->abort(ex);
    }
    return result;
}

void
IceProxy::RoboCompHumanTracker::HumanTracker::end_getUser(::RoboCompHumanTracker::TPerson& iceP_user, const ::Ice::AsyncResultPtr& result)
{
    ::Ice::AsyncResult::_check(result, this, iceC_RoboCompHumanTracker_HumanTracker_getUser_name);
    if(!result->_waitForResponse())
    {
        try
        {
            result->_throwUserException();
        }
        catch(const ::Ice::UserException& ex)
        {
            throw ::Ice::UnknownUserException(__FILE__, __LINE__, ex.ice_id());
        }
    }
    ::Ice::InputStream* istr = result->_startReadParams();
    istr->read(iceP_user);
    result->_endReadParams();
}

::Ice::AsyncResultPtr
IceProxy::RoboCompHumanTracker::HumanTracker::_iceI_begin_getUsersList(const ::Ice::Context& context, const ::IceInternal::CallbackBasePtr& del, const ::Ice::LocalObjectPtr& cookie, bool sync)
{
    _checkTwowayOnly(iceC_RoboCompHumanTracker_HumanTracker_getUsersList_name, sync);
    ::IceInternal::OutgoingAsyncPtr result = new ::IceInternal::CallbackOutgoing(this, iceC_RoboCompHumanTracker_HumanTracker_getUsersList_name, del, cookie, sync);
    try
    {
        result->prepare(iceC_RoboCompHumanTracker_HumanTracker_getUsersList_name, ::Ice::Normal, context);
        result->writeEmptyParams();
        result->invoke(iceC_RoboCompHumanTracker_HumanTracker_getUsersList_name);
    }
    catch(const ::Ice::Exception& ex)
    {
        result->abort(ex);
    }
    return result;
}

void
IceProxy::RoboCompHumanTracker::HumanTracker::end_getUsersList(::RoboCompHumanTracker::PersonList& iceP_users, const ::Ice::AsyncResultPtr& result)
{
    ::Ice::AsyncResult::_check(result, this, iceC_RoboCompHumanTracker_HumanTracker_getUsersList_name);
    if(!result->_waitForResponse())
    {
        try
        {
            result->_throwUserException();
        }
        catch(const ::Ice::UserException& ex)
        {
            throw ::Ice::UnknownUserException(__FILE__, __LINE__, ex.ice_id());
        }
    }
    ::Ice::InputStream* istr = result->_startReadParams();
    istr->read(iceP_users);
    result->_endReadParams();
}

::Ice::AsyncResultPtr
IceProxy::RoboCompHumanTracker::HumanTracker::_iceI_begin_getJointDepthPosition(::Ice::Int iceP_idperson, const ::std::string& iceP_idjoint, const ::Ice::Context& context, const ::IceInternal::CallbackBasePtr& del, const ::Ice::LocalObjectPtr& cookie, bool sync)
{
    _checkTwowayOnly(iceC_RoboCompHumanTracker_HumanTracker_getJointDepthPosition_name, sync);
    ::IceInternal::OutgoingAsyncPtr result = new ::IceInternal::CallbackOutgoing(this, iceC_RoboCompHumanTracker_HumanTracker_getJointDepthPosition_name, del, cookie, sync);
    try
    {
        result->prepare(iceC_RoboCompHumanTracker_HumanTracker_getJointDepthPosition_name, ::Ice::Normal, context);
        ::Ice::OutputStream* ostr = result->startWriteParams(::Ice::DefaultFormat);
        ostr->write(iceP_idperson);
        ostr->write(iceP_idjoint);
        result->endWriteParams();
        result->invoke(iceC_RoboCompHumanTracker_HumanTracker_getJointDepthPosition_name);
    }
    catch(const ::Ice::Exception& ex)
    {
        result->abort(ex);
    }
    return result;
}

bool
IceProxy::RoboCompHumanTracker::HumanTracker::end_getJointDepthPosition(::RoboCompHumanTracker::joint& iceP_depthjoint, const ::Ice::AsyncResultPtr& result)
{
    ::Ice::AsyncResult::_check(result, this, iceC_RoboCompHumanTracker_HumanTracker_getJointDepthPosition_name);
    bool ret;
    if(!result->_waitForResponse())
    {
        try
        {
            result->_throwUserException();
        }
        catch(const ::Ice::UserException& ex)
        {
            throw ::Ice::UnknownUserException(__FILE__, __LINE__, ex.ice_id());
        }
    }
    ::Ice::InputStream* istr = result->_startReadParams();
    istr->read(iceP_depthjoint);
    istr->read(ret);
    result->_endReadParams();
    return ret;
}

::IceProxy::Ice::Object*
IceProxy::RoboCompHumanTracker::HumanTracker::_newInstance() const
{
    return new HumanTracker;
}

const ::std::string&
IceProxy::RoboCompHumanTracker::HumanTracker::ice_staticId()
{
    return ::RoboCompHumanTracker::HumanTracker::ice_staticId();
}

RoboCompHumanTracker::HumanTracker::~HumanTracker()
{
}

::Ice::Object* RoboCompHumanTracker::upCast(::RoboCompHumanTracker::HumanTracker* p) { return p; }


namespace
{
const ::std::string iceC_RoboCompHumanTracker_HumanTracker_ids[2] =
{
    "::Ice::Object",
    "::RoboCompHumanTracker::HumanTracker"
};

}

bool
RoboCompHumanTracker::HumanTracker::ice_isA(const ::std::string& s, const ::Ice::Current&) const
{
    return ::std::binary_search(iceC_RoboCompHumanTracker_HumanTracker_ids, iceC_RoboCompHumanTracker_HumanTracker_ids + 2, s);
}

::std::vector< ::std::string>
RoboCompHumanTracker::HumanTracker::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&iceC_RoboCompHumanTracker_HumanTracker_ids[0], &iceC_RoboCompHumanTracker_HumanTracker_ids[2]);
}

const ::std::string&
RoboCompHumanTracker::HumanTracker::ice_id(const ::Ice::Current&) const
{
    return ice_staticId();
}

const ::std::string&
RoboCompHumanTracker::HumanTracker::ice_staticId()
{
#ifdef ICE_HAS_THREAD_SAFE_LOCAL_STATIC
    static const ::std::string typeId = "::RoboCompHumanTracker::HumanTracker";
    return typeId;
#else
    return iceC_RoboCompHumanTracker_HumanTracker_ids[1];
#endif
}

bool
RoboCompHumanTracker::HumanTracker::_iceD_getJointsPosition(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::Normal, current.mode);
    ::Ice::InputStream* istr = inS.startReadParams();
    ::Ice::Int iceP_id;
    istr->read(iceP_id);
    inS.endReadParams();
    ::RoboCompHumanTracker::jointListType iceP_jointList;
    this->getJointsPosition(iceP_id, iceP_jointList, current);
    ::Ice::OutputStream* ostr = inS.startWriteParams();
    ostr->write(iceP_jointList);
    inS.endWriteParams();
    return true;
}

bool
RoboCompHumanTracker::HumanTracker::_iceD_getRTMatrixList(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::Normal, current.mode);
    ::Ice::InputStream* istr = inS.startReadParams();
    ::Ice::Int iceP_id;
    istr->read(iceP_id);
    inS.endReadParams();
    ::RoboCompHumanTracker::RTMatrixList iceP_RTMatList;
    this->getRTMatrixList(iceP_id, iceP_RTMatList, current);
    ::Ice::OutputStream* ostr = inS.startWriteParams();
    ostr->write(iceP_RTMatList);
    inS.endWriteParams();
    return true;
}

bool
RoboCompHumanTracker::HumanTracker::_iceD_getUserState(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::Normal, current.mode);
    ::Ice::InputStream* istr = inS.startReadParams();
    ::Ice::Int iceP_id;
    istr->read(iceP_id);
    inS.endReadParams();
    ::RoboCompHumanTracker::TrackingState iceP_state;
    this->getUserState(iceP_id, iceP_state, current);
    ::Ice::OutputStream* ostr = inS.startWriteParams();
    ostr->write(iceP_state);
    inS.endWriteParams();
    return true;
}

bool
RoboCompHumanTracker::HumanTracker::_iceD_getUser(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::Normal, current.mode);
    ::Ice::InputStream* istr = inS.startReadParams();
    ::Ice::Int iceP_id;
    istr->read(iceP_id);
    inS.endReadParams();
    ::RoboCompHumanTracker::TPerson iceP_user;
    this->getUser(iceP_id, iceP_user, current);
    ::Ice::OutputStream* ostr = inS.startWriteParams();
    ostr->write(iceP_user);
    inS.endWriteParams();
    return true;
}

bool
RoboCompHumanTracker::HumanTracker::_iceD_getUsersList(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::Normal, current.mode);
    inS.readEmptyParams();
    ::RoboCompHumanTracker::PersonList iceP_users;
    this->getUsersList(iceP_users, current);
    ::Ice::OutputStream* ostr = inS.startWriteParams();
    ostr->write(iceP_users);
    inS.endWriteParams();
    return true;
}

bool
RoboCompHumanTracker::HumanTracker::_iceD_getJointDepthPosition(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::Normal, current.mode);
    ::Ice::InputStream* istr = inS.startReadParams();
    ::Ice::Int iceP_idperson;
    ::std::string iceP_idjoint;
    istr->read(iceP_idperson);
    istr->read(iceP_idjoint);
    inS.endReadParams();
    ::RoboCompHumanTracker::joint iceP_depthjoint;
    bool ret = this->getJointDepthPosition(iceP_idperson, iceP_idjoint, iceP_depthjoint, current);
    ::Ice::OutputStream* ostr = inS.startWriteParams();
    ostr->write(iceP_depthjoint);
    ostr->write(ret);
    inS.endWriteParams();
    return true;
}

namespace
{
const ::std::string iceC_RoboCompHumanTracker_HumanTracker_all[] =
{
    "getJointDepthPosition",
    "getJointsPosition",
    "getRTMatrixList",
    "getUser",
    "getUserState",
    "getUsersList",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping"
};

}

bool
RoboCompHumanTracker::HumanTracker::_iceDispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair<const ::std::string*, const ::std::string*> r = ::std::equal_range(iceC_RoboCompHumanTracker_HumanTracker_all, iceC_RoboCompHumanTracker_HumanTracker_all + 10, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - iceC_RoboCompHumanTracker_HumanTracker_all)
    {
        case 0:
        {
            return _iceD_getJointDepthPosition(in, current);
        }
        case 1:
        {
            return _iceD_getJointsPosition(in, current);
        }
        case 2:
        {
            return _iceD_getRTMatrixList(in, current);
        }
        case 3:
        {
            return _iceD_getUser(in, current);
        }
        case 4:
        {
            return _iceD_getUserState(in, current);
        }
        case 5:
        {
            return _iceD_getUsersList(in, current);
        }
        case 6:
        {
            return _iceD_ice_id(in, current);
        }
        case 7:
        {
            return _iceD_ice_ids(in, current);
        }
        case 8:
        {
            return _iceD_ice_isA(in, current);
        }
        case 9:
        {
            return _iceD_ice_ping(in, current);
        }
        default:
        {
            assert(false);
            throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
        }
    }
}

void
RoboCompHumanTracker::HumanTracker::_iceWriteImpl(::Ice::OutputStream* ostr) const
{
    ostr->startSlice(ice_staticId(), -1, true);
    Ice::StreamWriter< ::RoboCompHumanTracker::HumanTracker, ::Ice::OutputStream>::write(ostr, *this);
    ostr->endSlice();
}

void
RoboCompHumanTracker::HumanTracker::_iceReadImpl(::Ice::InputStream* istr)
{
    istr->startSlice();
    Ice::StreamReader< ::RoboCompHumanTracker::HumanTracker, ::Ice::InputStream>::read(istr, *this);
    istr->endSlice();
}

void
RoboCompHumanTracker::_icePatchObjectPtr(HumanTrackerPtr& handle, const ::Ice::ObjectPtr& v)
{
    handle = ::RoboCompHumanTracker::HumanTrackerPtr::dynamicCast(v);
    if(v && !handle)
    {
        IceInternal::Ex::throwUOE(::RoboCompHumanTracker::HumanTracker::ice_staticId(), v);
    }
}

namespace Ice
{
}

#endif