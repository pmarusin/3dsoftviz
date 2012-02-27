#ifndef __3DVisualWelcomeExecutor_H__
#define __3DVisualWelcomeExecutor_H__

#include "Network/executors/AbstractExecutor.h"
#include <QRegExp>

namespace Network {

    class WelcomeExecutor : public AbstractExecutor {

    private:
        QRegExp regexp;

    public:
        WelcomeExecutor(QRegExp regex);
        void execute();
        void setVariables(QRegExp new_regexp) {regexp=new_regexp;}

    };

}

#endif
