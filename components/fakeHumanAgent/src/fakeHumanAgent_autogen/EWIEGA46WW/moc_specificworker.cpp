/****************************************************************************
** Meta object code from reading C++ file 'specificworker.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../specificworker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'specificworker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SpecificWorker[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      23,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x0a,
      26,   15,   15,   15, 0x0a,
      36,   15,   15,   15, 0x0a,
      51,   15,   15,   15, 0x0a,
      63,   15,   15,   15, 0x0a,
      75,   15,   15,   15, 0x0a,
      88,   15,   15,   15, 0x0a,
     107,  101,   15,   15, 0x0a,
     126,  101,   15,   15, 0x0a,
     150,   15,   15,   15, 0x0a,
     167,   15,   15,   15, 0x0a,
     183,   15,   15,   15, 0x0a,
     189,   15,   15,   15, 0x0a,
     195,   15,   15,   15, 0x0a,
     203,   15,   15,   15, 0x0a,
     211,   15,   15,   15, 0x0a,
     220,   15,   15,   15, 0x0a,
     229,   15,   15,   15, 0x0a,
     237,   15,   15,   15, 0x0a,
     251,  245,   15,   15, 0x0a,
     262,   15,   15,   15, 0x0a,
     270,   15,   15,   15, 0x0a,
     278,   15,   15,   15, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SpecificWorker[] = {
    "SpecificWorker\0\0compute()\0setPose()\0"
    "autoMovement()\0addPerson()\0delPerson()\0"
    "savePoints()\0loadPoints()\0index\0"
    "personChanged(int)\0interactionChanged(int)\0"
    "addInteraction()\0removeEdgeAGM()\0upP()\0"
    "upR()\0downP()\0downR()\0rightP()\0rightR()\0"
    "leftP()\0leftR()\0valor\0rotar(int)\0"
    "giroP()\0giroR()\0moverandom()\0"
};

void SpecificWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SpecificWorker *_t = static_cast<SpecificWorker *>(_o);
        switch (_id) {
        case 0: _t->compute(); break;
        case 1: _t->setPose(); break;
        case 2: _t->autoMovement(); break;
        case 3: _t->addPerson(); break;
        case 4: _t->delPerson(); break;
        case 5: _t->savePoints(); break;
        case 6: _t->loadPoints(); break;
        case 7: _t->personChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->interactionChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->addInteraction(); break;
        case 10: _t->removeEdgeAGM(); break;
        case 11: _t->upP(); break;
        case 12: _t->upR(); break;
        case 13: _t->downP(); break;
        case 14: _t->downR(); break;
        case 15: _t->rightP(); break;
        case 16: _t->rightR(); break;
        case 17: _t->leftP(); break;
        case 18: _t->leftR(); break;
        case 19: _t->rotar((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 20: _t->giroP(); break;
        case 21: _t->giroR(); break;
        case 22: _t->moverandom(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SpecificWorker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SpecificWorker::staticMetaObject = {
    { &GenericWorker::staticMetaObject, qt_meta_stringdata_SpecificWorker,
      qt_meta_data_SpecificWorker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SpecificWorker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SpecificWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SpecificWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SpecificWorker))
        return static_cast<void*>(const_cast< SpecificWorker*>(this));
    return GenericWorker::qt_metacast(_clname);
}

int SpecificWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GenericWorker::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 23)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 23;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
