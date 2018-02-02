/****************************************************************************
** Meta object code from reading C++ file 'sigwatch.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../classes/sigwatch/sigwatch.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'sigwatch.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_UnixSignalWatcher[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      26,   19,   18,   18, 0x05,

 // slots: signature, parameters, type, tag, flags
      42,   18,   18,   18, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_UnixSignalWatcher[] = {
    "UnixSignalWatcher\0\0signal\0unixSignal(int)\0"
    "_q_onNotify(int)\0"
};

void UnixSignalWatcher::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        UnixSignalWatcher *_t = static_cast<UnixSignalWatcher *>(_o);
        switch (_id) {
        case 0: _t->unixSignal((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->d_func()->_q_onNotify((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData UnixSignalWatcher::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject UnixSignalWatcher::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_UnixSignalWatcher,
      qt_meta_data_UnixSignalWatcher, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UnixSignalWatcher::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UnixSignalWatcher::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UnixSignalWatcher::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UnixSignalWatcher))
        return static_cast<void*>(const_cast< UnixSignalWatcher*>(this));
    return QObject::qt_metacast(_clname);
}

int UnixSignalWatcher::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void UnixSignalWatcher::unixSignal(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
