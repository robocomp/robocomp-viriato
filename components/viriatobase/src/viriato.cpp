#include <viriato.h>

#include <QMutexLocker>

Viriato::Viriato(std::string port)
{
    mutex = new QMutex(QMutex::Recursive);
    string response = "";
    status = device.Connect(port.c_str());

    if (status != RQ_SUCCESS)
    {
        cout<<"Error connecting to device: "<<status<<"."<<endl;
        throw 1;
    }
}


Viriato::~Viriato()
{
    device.Disconnect();
}

void Viriato::setVelocity(float V1, float V2, float V3, float V4, float &rV1, float &rV2, float &rV3, float &rV4)
{
	device.SetCommand(false, _GO, 1, V1);
	device.SetCommand(false, _GO, 2, V2);
	device.SetCommand(true,  _GO, 1, V3);
	device.SetCommand(true,  _GO, 2, V4);
	int V1i=0, V2i=0, V3i=0, V4i=0;
	device.GetValue(false, _ABCNTR, 1, V1i);
	device.GetValue(false, _ABCNTR, 2, V2i);
	device.GetValue(true,  _ABCNTR, 1, V3i);
	device.GetValue(true,  _ABCNTR, 2, V4i);
	rV1 = V1i;
	rV2 = V2i;
	rV3 = V3i;
	rV4 = V4i;
// 	cout << "Got: " << V1i << " " << V2i << " " << V3i << " " << V4i << "\n";
}





