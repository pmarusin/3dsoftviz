#include "Network/executors/SetNodeColorExecutor.h"
#include "Manager/Manager.h"

using namespace Network;

void SetNodeColorExecutor::execute_client() {
    int id;

    float r, g, b, alpha;

    *stream >> id >> r >> g >> b >> alpha;

    Data::Graph * currentGraph = Manager::GraphManager::getInstance()->getActiveGraph();
    QMap<qlonglong, osg::ref_ptr<Data::Node> >* nodes = currentGraph -> getNodes();
    if (nodes->contains(id)) {
        Data::Node *node = *nodes->find(id);
        node->setColor(osg::Vec4(r,g,b,alpha));
        qDebug() << "Klient: nastavujem si farbu" << r << g << b << alpha;
    }

}

void SetNodeColorExecutor::execute_server() {
    int id;
    float r, g, b, alpha;

    *stream >> id >> r >> g >> b >> alpha;

    Data::Graph * currentGraph = Manager::GraphManager::getInstance()->getActiveGraph();
    QMap<qlonglong, osg::ref_ptr<Data::Node> >* nodes = currentGraph -> getNodes();
    if (nodes->contains(id)) {
        Data::Node *node = *nodes->find(id);
        node->setColor(osg::Vec4(r,g,b,alpha));
        qDebug() << "Server: nastavujem si farbu" << r << g << b << alpha;
    }

    Server * server = Server::getInstance();
    server->sendNodeColor(id, r, g, b, alpha);

    if (((QOSG::CoreWindow *)server->getCoreWindowReference())->playing()) {
        server->getLayoutThread()->play();
    }

}
