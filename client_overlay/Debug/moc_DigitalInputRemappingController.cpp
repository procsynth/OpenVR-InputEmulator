/****************************************************************************
** Meta object code from reading C++ file 'DigitalInputRemappingController.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/tabcontrollers/DigitalInputRemappingController.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'DigitalInputRemappingController.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_inputemulator__DigitalInputRemappingController_t {
    QByteArrayData data[46];
    char stringdata0[981];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_inputemulator__DigitalInputRemappingController_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_inputemulator__DigitalInputRemappingController_t qt_meta_stringdata_inputemulator__DigitalInputRemappingController = {
    {
QT_MOC_LITERAL(0, 0, 46), // "inputemulator::DigitalInputRe..."
QT_MOC_LITERAL(1, 47, 31), // "configureDigitalBindingFinished"
QT_MOC_LITERAL(2, 79, 0), // ""
QT_MOC_LITERAL(3, 80, 31), // "finishConfigureBinding_Original"
QT_MOC_LITERAL(4, 112, 31), // "finishConfigureBinding_Disabled"
QT_MOC_LITERAL(5, 144, 29), // "finishConfigureBinding_OpenVR"
QT_MOC_LITERAL(6, 174, 12), // "controllerId"
QT_MOC_LITERAL(7, 187, 8), // "ButtonId"
QT_MOC_LITERAL(8, 196, 10), // "toggleMode"
QT_MOC_LITERAL(9, 207, 15), // "toggleThreshold"
QT_MOC_LITERAL(10, 223, 11), // "autoTrigger"
QT_MOC_LITERAL(11, 235, 16), // "triggerFrequency"
QT_MOC_LITERAL(12, 252, 31), // "finishConfigureBinding_keyboard"
QT_MOC_LITERAL(13, 284, 12), // "shiftPressed"
QT_MOC_LITERAL(14, 297, 11), // "ctrlPressed"
QT_MOC_LITERAL(15, 309, 10), // "altPressed"
QT_MOC_LITERAL(16, 320, 8), // "keyIndex"
QT_MOC_LITERAL(17, 329, 42), // "finishConfigureBinding_suspen..."
QT_MOC_LITERAL(18, 372, 27), // "startConfigureNormalBinding"
QT_MOC_LITERAL(19, 400, 30), // "startConfigureLongPressBinding"
QT_MOC_LITERAL(20, 431, 32), // "startConfigureDoublePressBinding"
QT_MOC_LITERAL(21, 464, 22), // "getNormalBindingStatus"
QT_MOC_LITERAL(22, 487, 18), // "isLongPressEnabled"
QT_MOC_LITERAL(23, 506, 21), // "getLongPressThreshold"
QT_MOC_LITERAL(24, 528, 25), // "getLongPressBindingStatus"
QT_MOC_LITERAL(25, 554, 27), // "isLongPressImmediateRelease"
QT_MOC_LITERAL(26, 582, 20), // "isDoublePressEnabled"
QT_MOC_LITERAL(27, 603, 23), // "getDoublePressThreshold"
QT_MOC_LITERAL(28, 627, 27), // "getDoublePressBindingStatus"
QT_MOC_LITERAL(29, 655, 29), // "isDoublePressImmediateRelease"
QT_MOC_LITERAL(30, 685, 17), // "getButtonMaxCount"
QT_MOC_LITERAL(31, 703, 13), // "getButtonName"
QT_MOC_LITERAL(32, 717, 2), // "id"
QT_MOC_LITERAL(33, 720, 12), // "withDefaults"
QT_MOC_LITERAL(34, 733, 14), // "getBindingType"
QT_MOC_LITERAL(35, 748, 28), // "getBindingOpenVRControllerId"
QT_MOC_LITERAL(36, 777, 24), // "getBindingOpenVRButtonId"
QT_MOC_LITERAL(37, 802, 19), // "touchAsClickEnabled"
QT_MOC_LITERAL(38, 822, 19), // "isToggleModeEnabled"
QT_MOC_LITERAL(39, 842, 19), // "toggleModeThreshold"
QT_MOC_LITERAL(40, 862, 20), // "isAutoTriggerEnabled"
QT_MOC_LITERAL(41, 883, 20), // "autoTriggerFrequency"
QT_MOC_LITERAL(42, 904, 20), // "keyboardShiftEnabled"
QT_MOC_LITERAL(43, 925, 19), // "keyboardCtrlEnabled"
QT_MOC_LITERAL(44, 945, 18), // "keyboardAltEnabled"
QT_MOC_LITERAL(45, 964, 16) // "keyboardKeyIndex"

    },
    "inputemulator::DigitalInputRemappingController\0"
    "configureDigitalBindingFinished\0\0"
    "finishConfigureBinding_Original\0"
    "finishConfigureBinding_Disabled\0"
    "finishConfigureBinding_OpenVR\0"
    "controllerId\0ButtonId\0toggleMode\0"
    "toggleThreshold\0autoTrigger\0"
    "triggerFrequency\0finishConfigureBinding_keyboard\0"
    "shiftPressed\0ctrlPressed\0altPressed\0"
    "keyIndex\0finishConfigureBinding_suspendRedirectMode\0"
    "startConfigureNormalBinding\0"
    "startConfigureLongPressBinding\0"
    "startConfigureDoublePressBinding\0"
    "getNormalBindingStatus\0isLongPressEnabled\0"
    "getLongPressThreshold\0getLongPressBindingStatus\0"
    "isLongPressImmediateRelease\0"
    "isDoublePressEnabled\0getDoublePressThreshold\0"
    "getDoublePressBindingStatus\0"
    "isDoublePressImmediateRelease\0"
    "getButtonMaxCount\0getButtonName\0id\0"
    "withDefaults\0getBindingType\0"
    "getBindingOpenVRControllerId\0"
    "getBindingOpenVRButtonId\0touchAsClickEnabled\0"
    "isToggleModeEnabled\0toggleModeThreshold\0"
    "isAutoTriggerEnabled\0autoTriggerFrequency\0"
    "keyboardShiftEnabled\0keyboardCtrlEnabled\0"
    "keyboardAltEnabled\0keyboardKeyIndex"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_inputemulator__DigitalInputRemappingController[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      33,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,  179,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,  180,    2, 0x0a /* Public */,
       4,    0,  181,    2, 0x0a /* Public */,
       5,    6,  182,    2, 0x0a /* Public */,
      12,    8,  195,    2, 0x0a /* Public */,
      17,    0,  212,    2, 0x0a /* Public */,

 // methods: name, argc, parameters, tag, flags
      18,    0,  213,    2, 0x02 /* Public */,
      19,    0,  214,    2, 0x02 /* Public */,
      20,    0,  215,    2, 0x02 /* Public */,
      21,    0,  216,    2, 0x02 /* Public */,
      22,    0,  217,    2, 0x02 /* Public */,
      23,    0,  218,    2, 0x02 /* Public */,
      24,    0,  219,    2, 0x02 /* Public */,
      25,    0,  220,    2, 0x02 /* Public */,
      26,    0,  221,    2, 0x02 /* Public */,
      27,    0,  222,    2, 0x02 /* Public */,
      28,    0,  223,    2, 0x02 /* Public */,
      29,    0,  224,    2, 0x02 /* Public */,
      30,    0,  225,    2, 0x02 /* Public */,
      31,    2,  226,    2, 0x02 /* Public */,
      31,    1,  231,    2, 0x22 /* Public | MethodCloned */,
      34,    0,  234,    2, 0x02 /* Public */,
      35,    0,  235,    2, 0x02 /* Public */,
      36,    0,  236,    2, 0x02 /* Public */,
      37,    0,  237,    2, 0x02 /* Public */,
      38,    0,  238,    2, 0x02 /* Public */,
      39,    0,  239,    2, 0x02 /* Public */,
      40,    0,  240,    2, 0x02 /* Public */,
      41,    0,  241,    2, 0x02 /* Public */,
      42,    0,  242,    2, 0x02 /* Public */,
      43,    0,  243,    2, 0x02 /* Public */,
      44,    0,  244,    2, 0x02 /* Public */,
      45,    0,  245,    2, 0x02 /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Int, QMetaType::Bool, QMetaType::Int, QMetaType::Bool, QMetaType::Int,    6,    7,    8,    9,   10,   11,
    QMetaType::Void, QMetaType::Bool, QMetaType::Bool, QMetaType::Bool, QMetaType::ULong, QMetaType::Bool, QMetaType::Int, QMetaType::Bool, QMetaType::Int,   13,   14,   15,   16,    8,    9,   10,   11,
    QMetaType::Void,

 // methods: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::QString,
    QMetaType::Bool,
    QMetaType::UInt,
    QMetaType::QString,
    QMetaType::Bool,
    QMetaType::Bool,
    QMetaType::UInt,
    QMetaType::QString,
    QMetaType::Bool,
    QMetaType::Int,
    QMetaType::QString, QMetaType::Int, QMetaType::Bool,   32,   33,
    QMetaType::QString, QMetaType::Int,   32,
    QMetaType::Int,
    QMetaType::Int,
    QMetaType::Int,
    QMetaType::Bool,
    QMetaType::Bool,
    QMetaType::Int,
    QMetaType::Bool,
    QMetaType::Int,
    QMetaType::Bool,
    QMetaType::Bool,
    QMetaType::Bool,
    QMetaType::UInt,

       0        // eod
};

void inputemulator::DigitalInputRemappingController::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DigitalInputRemappingController *_t = static_cast<DigitalInputRemappingController *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->configureDigitalBindingFinished(); break;
        case 1: _t->finishConfigureBinding_Original(); break;
        case 2: _t->finishConfigureBinding_Disabled(); break;
        case 3: _t->finishConfigureBinding_OpenVR((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5])),(*reinterpret_cast< int(*)>(_a[6]))); break;
        case 4: _t->finishConfigureBinding_keyboard((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< ulong(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5])),(*reinterpret_cast< int(*)>(_a[6])),(*reinterpret_cast< bool(*)>(_a[7])),(*reinterpret_cast< int(*)>(_a[8]))); break;
        case 5: _t->finishConfigureBinding_suspendRedirectMode(); break;
        case 6: _t->startConfigureNormalBinding(); break;
        case 7: _t->startConfigureLongPressBinding(); break;
        case 8: _t->startConfigureDoublePressBinding(); break;
        case 9: { QString _r = _t->getNormalBindingStatus();
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = std::move(_r); }  break;
        case 10: { bool _r = _t->isLongPressEnabled();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 11: { uint _r = _t->getLongPressThreshold();
            if (_a[0]) *reinterpret_cast< uint*>(_a[0]) = std::move(_r); }  break;
        case 12: { QString _r = _t->getLongPressBindingStatus();
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = std::move(_r); }  break;
        case 13: { bool _r = _t->isLongPressImmediateRelease();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 14: { bool _r = _t->isDoublePressEnabled();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 15: { uint _r = _t->getDoublePressThreshold();
            if (_a[0]) *reinterpret_cast< uint*>(_a[0]) = std::move(_r); }  break;
        case 16: { QString _r = _t->getDoublePressBindingStatus();
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = std::move(_r); }  break;
        case 17: { bool _r = _t->isDoublePressImmediateRelease();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 18: { int _r = _t->getButtonMaxCount();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 19: { QString _r = _t->getButtonName((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = std::move(_r); }  break;
        case 20: { QString _r = _t->getButtonName((*reinterpret_cast< int(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = std::move(_r); }  break;
        case 21: { int _r = _t->getBindingType();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 22: { int _r = _t->getBindingOpenVRControllerId();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 23: { int _r = _t->getBindingOpenVRButtonId();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 24: { bool _r = _t->touchAsClickEnabled();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 25: { bool _r = _t->isToggleModeEnabled();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 26: { int _r = _t->toggleModeThreshold();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 27: { bool _r = _t->isAutoTriggerEnabled();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 28: { int _r = _t->autoTriggerFrequency();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 29: { bool _r = _t->keyboardShiftEnabled();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 30: { bool _r = _t->keyboardCtrlEnabled();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 31: { bool _r = _t->keyboardAltEnabled();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 32: { uint _r = _t->keyboardKeyIndex();
            if (_a[0]) *reinterpret_cast< uint*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (DigitalInputRemappingController::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DigitalInputRemappingController::configureDigitalBindingFinished)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject inputemulator::DigitalInputRemappingController::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_inputemulator__DigitalInputRemappingController.data,
      qt_meta_data_inputemulator__DigitalInputRemappingController,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *inputemulator::DigitalInputRemappingController::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *inputemulator::DigitalInputRemappingController::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_inputemulator__DigitalInputRemappingController.stringdata0))
        return static_cast<void*>(const_cast< DigitalInputRemappingController*>(this));
    return QObject::qt_metacast(_clname);
}

int inputemulator::DigitalInputRemappingController::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 33)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 33;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 33)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 33;
    }
    return _id;
}

// SIGNAL 0
void inputemulator::DigitalInputRemappingController::configureDigitalBindingFinished()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
