#include "Network/executors/ServerStopExecutor.h"
#include "Network/Client.h"

namespace Network {

void ServerStopExecutor::execute_client()
{

	Client* client = Client::getInstance();
	client->disconnect();

}

void ServerStopExecutor::execute_server()
{
	return;
}

} // namespace Network
