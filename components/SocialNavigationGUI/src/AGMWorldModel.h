//
// Copyright (c) ZeroC, Inc. All rights reserved.
//
//
// Ice version 3.7.3
//
// <auto-generated>
//
// Generated from file `AGMWorldModel.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef __AGMWorldModel_h__
#define __AGMWorldModel_h__

#include <IceUtil/PushDisableWarnings.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/ValueF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Comparable.h>
#include <IceUtil/ScopedArray.h>
#include <Ice/Optional.h>
#include <IceUtil/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 307
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 >= 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 3
#       error Ice patch level mismatch!
#   endif
#endif

#ifdef ICE_CPP11_MAPPING // C++11 mapping

namespace RoboCompAGMWorldModel
{

using StringDictionary = ::std::map<::std::string, ::std::string>;

enum class BehaviorResultType : unsigned char
{
    InitialWorld,
    BehaviorBasedModification,
    BehaviorBasedNumericUpdate,
    StatusFailTimeout,
    StatusFailOther,
    StatusActive,
    StatusIdle
};

struct Node
{
    ::std::string nodeType;
    int nodeIdentifier;
    ::RoboCompAGMWorldModel::StringDictionary attributes;

    /**
     * Obtains a tuple containing all of the struct's data members.
     * @return The data members in a tuple.
     */
    std::tuple<const ::std::string&, const int&, const ::RoboCompAGMWorldModel::StringDictionary&> ice_tuple() const
    {
        return std::tie(nodeType, nodeIdentifier, attributes);
    }
};

using NodeSequence = ::std::vector<Node>;

struct Edge
{
    int a;
    int b;
    ::std::string edgeType;
    ::RoboCompAGMWorldModel::StringDictionary attributes;

    /**
     * Obtains a tuple containing all of the struct's data members.
     * @return The data members in a tuple.
     */
    std::tuple<const int&, const int&, const ::std::string&, const ::RoboCompAGMWorldModel::StringDictionary&> ice_tuple() const
    {
        return std::tie(a, b, edgeType, attributes);
    }
};

using EdgeSequence = ::std::vector<Edge>;

struct World
{
    ::RoboCompAGMWorldModel::NodeSequence nodes;
    ::RoboCompAGMWorldModel::EdgeSequence edges;
    int version;

    /**
     * Obtains a tuple containing all of the struct's data members.
     * @return The data members in a tuple.
     */
    std::tuple<const ::RoboCompAGMWorldModel::NodeSequence&, const ::RoboCompAGMWorldModel::EdgeSequence&, const int&> ice_tuple() const
    {
        return std::tie(nodes, edges, version);
    }
};

struct Event
{
    ::std::string sender;
    ::RoboCompAGMWorldModel::BehaviorResultType why;
    ::RoboCompAGMWorldModel::World backModel;
    ::RoboCompAGMWorldModel::World newModel;

    /**
     * Obtains a tuple containing all of the struct's data members.
     * @return The data members in a tuple.
     */
    std::tuple<const ::std::string&, const ::RoboCompAGMWorldModel::BehaviorResultType&, const ::RoboCompAGMWorldModel::World&, const ::RoboCompAGMWorldModel::World&> ice_tuple() const
    {
        return std::tie(sender, why, backModel, newModel);
    }
};

using Ice::operator<;
using Ice::operator<=;
using Ice::operator>;
using Ice::operator>=;
using Ice::operator==;
using Ice::operator!=;

}

/// \cond STREAM
namespace Ice
{

template<>
struct StreamableTraits< ::RoboCompAGMWorldModel::BehaviorResultType>
{
    static const StreamHelperCategory helper = StreamHelperCategoryEnum;
    static const int minValue = 0;
    static const int maxValue = 6;
    static const int minWireSize = 1;
    static const bool fixedLength = false;
};

template<>
struct StreamableTraits<::RoboCompAGMWorldModel::Node>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 6;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamReader<::RoboCompAGMWorldModel::Node, S>
{
    static void read(S* istr, ::RoboCompAGMWorldModel::Node& v)
    {
        istr->readAll(v.nodeType, v.nodeIdentifier, v.attributes);
    }
};

template<>
struct StreamableTraits<::RoboCompAGMWorldModel::Edge>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 10;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamReader<::RoboCompAGMWorldModel::Edge, S>
{
    static void read(S* istr, ::RoboCompAGMWorldModel::Edge& v)
    {
        istr->readAll(v.a, v.b, v.edgeType, v.attributes);
    }
};

template<>
struct StreamableTraits<::RoboCompAGMWorldModel::World>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 6;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamReader<::RoboCompAGMWorldModel::World, S>
{
    static void read(S* istr, ::RoboCompAGMWorldModel::World& v)
    {
        istr->readAll(v.nodes, v.edges, v.version);
    }
};

template<>
struct StreamableTraits<::RoboCompAGMWorldModel::Event>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 14;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamReader<::RoboCompAGMWorldModel::Event, S>
{
    static void read(S* istr, ::RoboCompAGMWorldModel::Event& v)
    {
        istr->readAll(v.sender, v.why, v.backModel, v.newModel);
    }
};

}
/// \endcond

#else // C++98 mapping

namespace RoboCompAGMWorldModel
{

typedef ::std::map< ::std::string, ::std::string> StringDictionary;

enum BehaviorResultType
{
    InitialWorld,
    BehaviorBasedModification,
    BehaviorBasedNumericUpdate,
    StatusFailTimeout,
    StatusFailOther,
    StatusActive,
    StatusIdle
};

struct Node
{
    ::std::string nodeType;
    ::Ice::Int nodeIdentifier;
    ::RoboCompAGMWorldModel::StringDictionary attributes;
};

typedef ::std::vector<Node> NodeSequence;

struct Edge
{
    ::Ice::Int a;
    ::Ice::Int b;
    ::std::string edgeType;
    ::RoboCompAGMWorldModel::StringDictionary attributes;
};

typedef ::std::vector<Edge> EdgeSequence;

struct World
{
    ::RoboCompAGMWorldModel::NodeSequence nodes;
    ::RoboCompAGMWorldModel::EdgeSequence edges;
    ::Ice::Int version;
};

struct Event
{
    ::std::string sender;
    ::RoboCompAGMWorldModel::BehaviorResultType why;
    ::RoboCompAGMWorldModel::World backModel;
    ::RoboCompAGMWorldModel::World newModel;
};

}

/// \cond STREAM
namespace Ice
{

template<>
struct StreamableTraits< ::RoboCompAGMWorldModel::BehaviorResultType>
{
    static const StreamHelperCategory helper = StreamHelperCategoryEnum;
    static const int minValue = 0;
    static const int maxValue = 6;
    static const int minWireSize = 1;
    static const bool fixedLength = false;
};

template<>
struct StreamableTraits< ::RoboCompAGMWorldModel::Node>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 6;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamWriter< ::RoboCompAGMWorldModel::Node, S>
{
    static void write(S* ostr, const ::RoboCompAGMWorldModel::Node& v)
    {
        ostr->write(v.nodeType);
        ostr->write(v.nodeIdentifier);
        ostr->write(v.attributes);
    }
};

template<typename S>
struct StreamReader< ::RoboCompAGMWorldModel::Node, S>
{
    static void read(S* istr, ::RoboCompAGMWorldModel::Node& v)
    {
        istr->read(v.nodeType);
        istr->read(v.nodeIdentifier);
        istr->read(v.attributes);
    }
};

template<>
struct StreamableTraits< ::RoboCompAGMWorldModel::Edge>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 10;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamWriter< ::RoboCompAGMWorldModel::Edge, S>
{
    static void write(S* ostr, const ::RoboCompAGMWorldModel::Edge& v)
    {
        ostr->write(v.a);
        ostr->write(v.b);
        ostr->write(v.edgeType);
        ostr->write(v.attributes);
    }
};

template<typename S>
struct StreamReader< ::RoboCompAGMWorldModel::Edge, S>
{
    static void read(S* istr, ::RoboCompAGMWorldModel::Edge& v)
    {
        istr->read(v.a);
        istr->read(v.b);
        istr->read(v.edgeType);
        istr->read(v.attributes);
    }
};

template<>
struct StreamableTraits< ::RoboCompAGMWorldModel::World>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 6;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamWriter< ::RoboCompAGMWorldModel::World, S>
{
    static void write(S* ostr, const ::RoboCompAGMWorldModel::World& v)
    {
        ostr->write(v.nodes);
        ostr->write(v.edges);
        ostr->write(v.version);
    }
};

template<typename S>
struct StreamReader< ::RoboCompAGMWorldModel::World, S>
{
    static void read(S* istr, ::RoboCompAGMWorldModel::World& v)
    {
        istr->read(v.nodes);
        istr->read(v.edges);
        istr->read(v.version);
    }
};

template<>
struct StreamableTraits< ::RoboCompAGMWorldModel::Event>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 14;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamWriter< ::RoboCompAGMWorldModel::Event, S>
{
    static void write(S* ostr, const ::RoboCompAGMWorldModel::Event& v)
    {
        ostr->write(v.sender);
        ostr->write(v.why);
        ostr->write(v.backModel);
        ostr->write(v.newModel);
    }
};

template<typename S>
struct StreamReader< ::RoboCompAGMWorldModel::Event, S>
{
    static void read(S* istr, ::RoboCompAGMWorldModel::Event& v)
    {
        istr->read(v.sender);
        istr->read(v.why);
        istr->read(v.backModel);
        istr->read(v.newModel);
    }
};

}
/// \endcond

#endif

#include <IceUtil/PopDisableWarnings.h>
#endif