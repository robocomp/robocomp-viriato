/****************************************************************************
** Meta object code from reading C++ file 'specificworker.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "specificworker.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'specificworker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SpecificWorker_t {
    QByteArrayData data[19];
    char stringdata0[248];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SpecificWorker_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SpecificWorker_t qt_meta_stringdata_SpecificWorker = {
    {
QT_MOC_LITERAL(0, 0, 14), // "SpecificWorker"
QT_MOC_LITERAL(1, 15, 7), // "compute"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 10), // "initialize"
QT_MOC_LITERAL(4, 35, 6), // "period"
QT_MOC_LITERAL(5, 42, 22), // "checkRobotAutoMovState"
QT_MOC_LITERAL(6, 65, 9), // "moveRobot"
QT_MOC_LITERAL(7, 75, 11), // "sendRobotTo"
QT_MOC_LITERAL(8, 87, 19), // "forcesSliderChanged"
QT_MOC_LITERAL(9, 107, 5), // "value"
QT_MOC_LITERAL(10, 113, 10), // "sm_compute"
QT_MOC_LITERAL(11, 124, 13), // "sm_initialize"
QT_MOC_LITERAL(12, 138, 11), // "sm_finalize"
QT_MOC_LITERAL(13, 150, 17), // "setPositionButton"
QT_MOC_LITERAL(14, 168, 19), // "startOptimizeButton"
QT_MOC_LITERAL(15, 188, 17), // "save_trainingData"
QT_MOC_LITERAL(16, 206, 11), // "std::string"
QT_MOC_LITERAL(17, 218, 15), // "restartPosition"
QT_MOC_LITERAL(18, 234, 13) // "secondCompute"

    },
    "SpecificWorker\0compute\0\0initialize\0"
    "period\0checkRobotAutoMovState\0moveRobot\0"
    "sendRobotTo\0forcesSliderChanged\0value\0"
    "sm_compute\0sm_initialize\0sm_finalize\0"
    "setPositionButton\0startOptimizeButton\0"
    "save_trainingData\0std::string\0"
    "restartPosition\0secondCompute"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SpecificWorker[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   89,    2, 0x0a /* Public */,
       3,    1,   90,    2, 0x0a /* Public */,
       5,    0,   93,    2, 0x0a /* Public */,
       6,    0,   94,    2, 0x0a /* Public */,
       7,    0,   95,    2, 0x0a /* Public */,
       8,    1,   96,    2, 0x0a /* Public */,
       8,    0,   99,    2, 0x2a /* Public | MethodCloned */,
      10,    0,  100,    2, 0x0a /* Public */,
      11,    0,  101,    2, 0x0a /* Public */,
      12,    0,  102,    2, 0x0a /* Public */,
      13,    0,  103,    2, 0x0a /* Public */,
      14,    0,  104,    2, 0x0a /* Public */,
      15,    1,  105,    2, 0x0a /* Public */,
      17,    0,  108,    2, 0x0a /* Public */,
      18,    0,  109,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 16,    2,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void SpecificWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SpecificWorker *_t = static_cast<SpecificWorker *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->compute(); break;
        case 1: _t->initialize((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->checkRobotAutoMovState(); break;
        case 3: _t->moveRobot(); break;
        case 4: _t->sendRobotTo(); break;
        case 5: _t->forcesSliderChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->forcesSliderChanged(); break;
        case 7: _t->sm_compute(); break;
        case 8: _t->sm_initialize(); break;
        case 9: _t->sm_finalize(); break;
        case 10: _t->setPositionButton(); break;
        case 11: _t->startOptimizeButton(); break;
        case 12: _t->save_trainingData((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 13: _t->restartPosition(); break;
        case 14: _t->secondCompute(); break;
        default: ;
        }
    }
}

const QMetaObject SpecificWorker::staticMetaObject = {
    { &GenericWorker::staticMetaObject, qt_meta_stringdata_SpecificWorker.data,
      qt_meta_data_SpecificWorker,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SpecificWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SpecificWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SpecificWorker.stringdata0))
        return static_cast<void*>(this);
    return GenericWorker::qt_metacast(_clname);
}

int SpecificWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GenericWorker::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 15;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
