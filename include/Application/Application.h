﻿#pragma once

#include <QApplication>
#include <QtGlobal>

#include <QDebug>
#include <QX11Info>

#ifdef Q_OS_LINUX
    #include <X11/Xlib.h>

    #undef None
    #undef Status
    #undef Unsorted
    #undef GrayScale

#endif

namespace App {

class Application : public QApplication {
	   Q_OBJECT

public:
   Application(int &argc, char **argv);

  ~Application();

#if defined(Q_OS_LINUX)

   virtual bool x11EventFilter(XEvent *event);

   void getX11Event(XEvent &event);

private:
   XEvent event;

#endif

};
} //App
