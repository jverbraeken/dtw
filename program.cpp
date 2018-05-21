#pragma comment(lib, "Ws2_32.lib")
#include <winsock2.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <chrono>
#include <time.h>
#include "dtw.h"
#include "CircularBuffer.h"

#define BUFLEN 512
#define PORT 55056

static const unsigned char COMTP_SENSOR_DATA = 1;
static const unsigned char COMTP_SHAKING_STARTED = 2;
static const unsigned char COMTP_SHAKING_STOPPED = 3;

float ERROR_THRESHOLD = 0.5;

static const unsigned int MINIMUM_TIME_BETWEEN_GESTURES = 1000;

static const unsigned int DATA_FREQUENCY = 25;

unsigned int numGestures;
// per gesture per sample per dimension
vector<vector<vector<vector<double>*>*>*> preGestureValues;
// per resizement per gesture per dimension per sample
// vector<vector<vector<vector<float>*>*>*> resizedAveragedPreGestureValues;

// per per gesture per sample per dimension a CircularBuffer
vector<vector<vector<CircularBuffer<double> *> *> *> dataBuffer;

// per gesture per sample the detection time
vector<vector<long long>*> gestureDetectionTime;
// per gesture per sample a bounty
vector<vector<int>*> gestureBounty;

//////// HELPER FUNCTIONS




unsigned char getByteFromBuffer(char *buf, int offset) {
	unsigned char result;
	memcpy(&result, &buf[offset], 1);

	return result;
}

float getFloatFromBuffer(char *buf, int offset) {
	float tmp;
	float result;
	memcpy(&tmp, &buf[offset], 4);

	char *floatToConvert = reinterpret_cast<char *>(&tmp);
	char *returnFloat = reinterpret_cast<char *>(&result);
	returnFloat[0] = floatToConvert[3];
	returnFloat[1] = floatToConvert[2];
	returnFloat[2] = floatToConvert[1];
	returnFloat[3] = floatToConvert[0];

	return result;
}

long long getLongFromBuffer(char *buf, int offset) {
	long long tmp;
	long long result;
	memcpy(&tmp, &buf[offset], 8);

	char *longToConvert = reinterpret_cast<char *>(&tmp);
	char *returnLong = reinterpret_cast<char *>(&result);
	returnLong[0] = longToConvert[7];
	returnLong[1] = longToConvert[6];
	returnLong[2] = longToConvert[5];
	returnLong[3] = longToConvert[4];
	returnLong[4] = longToConvert[3];
	returnLong[5] = longToConvert[2];
	returnLong[6] = longToConvert[1];
	returnLong[7] = longToConvert[0];

	return result;
}



void checkDataBuffer() {
	long long now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	for (int gesture = 0; gesture < 1 /*numGestures */; gesture++) {
		for (int sample = 0; sample < 1 /*(*preGestureValues[gesture]).size() */; sample++) {
			if ((*(gestureBounty[gesture]))[sample] > 0) {
				(*(gestureBounty[gesture]))[sample]--;
				continue;
			}
			Vector<double> dataX = (*(*dataBuffer[gesture])[sample])[0]->getData();
			Vector<double> dataY = (*(*dataBuffer[gesture])[sample])[1]->getData();
			Vector<double> dataZ = (*(*dataBuffer[gesture])[sample])[2]->getData();
			vector<double> newDataX;
			vector<double> newDataY;
			//std::cout << "now: " << now << ", other: " << (*(gestureDetectionTime[gesture]))[sample] << endl;
			if (now - (*(gestureDetectionTime[gesture]))[sample] > MINIMUM_TIME_BETWEEN_GESTURES) {
				double error = 0;
				int numValuesInBuffer = (*(*dataBuffer[gesture])[sample])[0]->getNumValuesInBuffer();
				int bufferSize = (*(*dataBuffer[gesture])[sample])[0]->getSize();
				if (numValuesInBuffer == bufferSize) {
					/*double startAngle = atan2(dataX[0], dataY[0]);
					double cs = cos(startAngle);
					double sn = sin(startAngle);
					for (int i = 0; i < dataX.size(); i++) {
						newDataX.push_back(dataX[i] * cs - dataY[i] * sn);
						newDataY.push_back(dataX[i] * sn + dataY[i] * cs);
					}*/
					newDataX = dataX;
					newDataY = dataY;
					LB_Improved filterX((*(*(*(preGestureValues[gesture]))[sample])[0]), 6);
					error += (filterX.test(newDataX) / (*(*(*(preGestureValues[gesture]))[sample])[0]).size());
					LB_Improved filterY((*(*(*(preGestureValues[gesture]))[sample])[1]), 6);
					error += (filterY.test(newDataY) / (*(*(*(preGestureValues[gesture]))[sample])[1]).size());
					LB_Improved filterZ((*(*(*(preGestureValues[gesture]))[sample])[2]), 6);
					error += (filterZ.test(dataZ) / (*(*(*(preGestureValues[gesture]))[sample])[2]).size());
					printf("%f\n", error);
					if (error < ERROR_THRESHOLD) {
						printf("\n\nGESTURE MATCHED %d-%d\n\n", gesture, sample);
						(*(gestureDetectionTime[gesture]))[sample] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
					}
					else {
						(*(gestureBounty[gesture]))[sample] = error < ERROR_THRESHOLD * 1.3 ? 1 : (int)((error - ERROR_THRESHOLD) * 2);
					}
				}			
			}
		}
	}
}



void useGRT() {
	preGestureValues = vector<vector<vector<vector<double>*>*>*>();

	cout << "Testing file \"golf.grt\"" << endl << endl;
	ifstream in;
	in.open("C:\\Users\\jverb\\Documents\\Git\\dtw-test\\golf.grt");
	if (!in.is_open()) {
		cout << "FILE CANNOT BE OPENED!!" << endl << endl;
		return;
	}

	string word;
	int numTrainingExamples;

	//Check to make sure this is a file with the Training File Format
	in >> word;
	if (word != "GRT_LABELLED_TIME_SERIES_CLASSIFICATION_DATA_FILE_V1.0") {
		cout << "loadDatasetFromFile(std::string filename) - Failed to find file header!" << std::endl;
	}

	//Get the name of the dataset
	in >> word;
	if (word != "DatasetName:") {
		cout << "loadDatasetFromFile(std::string filename) - failed to find DatasetName!" << std::endl;
	}
	in >> word;

	in >> word;
	if (word != "InfoText:") {
		cout << "loadDatasetFromFile(std::string filename) - failed to find InfoText!" << std::endl;
	}
	in >> word;

	//Get the total number of training examples in the training data
	in >> word;
	if (word != "TotalNumTrainingExamples:") {
		cout << "loadDatasetFromFile(std::string filename) - Failed to find TotalNumTrainingExamples!" << std::endl;
	}
	in >> numTrainingExamples;

	//Get the total number of classes in the training data
	in >> word;
	if (word != "NumberOfClasses:") {
		cout << "loadDatasetFromFile(std::string filename) - Failed to find NumberOfClasses!" << std::endl;
	}
	in >> numGestures;
	for (int x = 0; x < numGestures; x++) {
		preGestureValues.push_back(new vector<vector<vector<double>*>*>());
		dataBuffer.push_back(new vector<vector<CircularBuffer<double>*>*>());
		gestureDetectionTime.push_back(new vector<long long>());
		gestureBounty.push_back(new vector<int>());
	}

	//Get the total number of classes in the training data
	in >> word;
	if (word != "ClassIDsAndCounters:") {
		cout << "loadDatasetFromFile(std::string filename) - Failed to find ClassIDsAndCounters!" << std::endl;
	}

	for (int i = 0; i < numGestures; i++) {
		int clazz, counter;
		in >> clazz;
		in >> counter;
	}

	/*if (useExternalRanges) {
	externalRanges.resize(numDimensions);
	for (UINT i = 0; i<externalRanges.size(); i++) {
	in >> externalRanges[i].minValue;
	in >> externalRanges[i].maxValue;
	}
	}*/

	//Get the main training data
	in >> word;
	if (word != "LabelledTimeSeriesTrainingData:") {
		in.close();
		cout << "loadDatasetFromFile(std::string filename) - Failed to find LabelledTimeSeriesTrainingData!" << std::endl;
	}

	//Load each of the time series
	for (UINT i = 0; i < numTrainingExamples; i++) {
		UINT classLabel = 0;
		UINT timeSeriesLength = 0;
		int duration = 0;

		in >> word;
		if (word != "************TIME_SERIES************") {
			in.close();
			cout << "loadDatasetFromFile(std::string filename) - Failed to find TimeSeries Header!" << std::endl;
		}

		in >> word;
		if (word != "ClassID:") {
			in.close();
			cout << "loadDatasetFromFile(std::string filename) - Failed to find ClassID!" << std::endl;
		}
		in >> classLabel;
		classLabel--;

		in >> word;
		if (word != "TimeSeriesLength:") {
			in.close();
			cout << "loadDatasetFromFile(std::string filename) - Failed to find TimeSeriesLength!" << std::endl;
		}
		in >> timeSeriesLength;

		in >> word;
		if (word != "Duration:") {
			in.close();
			cout << "loadDatasetFromFile(std::string filename) - Failed to find Duration!" << std::endl;
		}
		in >> duration;

		in >> word;
		if (word != "TimeSeriesData:") {
			in.close();
			cout << "loadDatasetFromFile(std::string filename) - Failed to find TimeSeriesData!" << std::endl;
		}

		preGestureValues[classLabel]->push_back(new vector<vector<double>*>());
		dataBuffer[classLabel]->push_back(new vector<CircularBuffer<double>*>());

		for (int j = 0; j < 6; j++) {
			preGestureValues[classLabel]->back()->push_back(new vector<double>());
			dataBuffer[classLabel]->back()->push_back(new CircularBuffer<double>(timeSeriesLength));
		}

		gestureDetectionTime[classLabel]->push_back(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

		gestureBounty[classLabel]->push_back(25);

		//double startAngle;
		for (int j = 0; j < timeSeriesLength; j++) {
			float x, y, z, nx, ny;
			in >> x;
			in >> y;
			in >> z;
			x /= 100.f;
			y /= 100.f;
			z /= 100.f;
			//if (j == 0) {
			//	startAngle = atan2(x, y);
			//}
			//double cs = cos(startAngle);
			//double sn = sin(startAngle);
			//nx = x * cs - y * sn;
			//ny = x * sn + y * cs;
			nx = x;
			ny = y;
			(*preGestureValues[classLabel]->back())[0]->push_back(nx);
			(*preGestureValues[classLabel]->back())[1]->push_back(ny);
			(*preGestureValues[classLabel]->back())[2]->push_back(z);
			for (int k = 3; k < 6; k++) {
				float val;
				in >> val;
				(*preGestureValues[classLabel]->back())[k]->push_back(val / 100.f);
			}
		}
	}
	in.close();
}

int main() {
	cout << "UWF testing" << endl;

	//dataBuffer = new vector<vector<vector<CircularBuffer<double> *> *> *>();

	useGRT();







	SOCKET s;
	struct sockaddr_in server, si_other;
	char buf[BUFLEN];
	WSADATA wsa;

	int slen = sizeof(si_other);

	//Initialise winsock
	printf("Initialising Winsock...\n");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		printf("Failed. Error Code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	printf("Initialised.\n");

	//Create a socket
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET) {
		printf("Could not create socket : %d", WSAGetLastError());
	}
	printf("Socket created.\n");

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(PORT);

	//Bind
	if (bind(s, reinterpret_cast<struct sockaddr *>(&server), sizeof(server)) == SOCKET_ERROR) {
		printf("Bind failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	puts("Bind done");








	//keep listening for data
	while (true) {
		//printf("Waiting for data...");
		fflush(stdout);

		//clear the buffer by filling null, it might have previously received data
		memset(buf, '\0', BUFLEN);

		//try to receive some data, this is a blocking call
		if (recvfrom(s, buf, BUFLEN, 0, reinterpret_cast<struct sockaddr *>(&si_other), &slen) == SOCKET_ERROR) {
			printf("recvfrom() failed with error code : %d", WSAGetLastError());
			exit(EXIT_FAILURE);
		}

		//print details of the client/peer and the data received
		//printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
		//printf("Data: %s\n", buf);
		unsigned char request = getByteFromBuffer(buf, 0);
		if (request == COMTP_SENSOR_DATA) {
			float xvector = getFloatFromBuffer(buf, 1);
			float yvector = getFloatFromBuffer(buf, 5);
			float zvector = getFloatFromBuffer(buf, 9);
			//float xrot = getFloatFromBuffer(buf, 13);
			//float yrot = getFloatFromBuffer(buf, 17);
			//float zrot = getFloatFromBuffer(buf, 21);
			//long long rot_timestamp = getLongFromBuffer(buf, 25);

			float xacc = getFloatFromBuffer(buf, 33);
			float yacc = getFloatFromBuffer(buf, 37);
			float zacc = getFloatFromBuffer(buf, 41);
			//long long acc_timestamp = getLongFromBuffer(buf, 45);

			vector<float> data = vector<float>();
			data.push_back(xvector);
			data.push_back(yvector);
			data.push_back(zvector);
			data.push_back(xacc);
			data.push_back(yacc);
			data.push_back(zacc);

			for (int i = 0; i < dataBuffer.size(); i++) {
				for (int j = 0; j < dataBuffer[i]->size(); j++) {
					for (int k = 0; k < 6; k++) {
						(*(*dataBuffer[i])[j])[k]->push_back(data[k]);
					}
				}
			}

			checkDataBuffer();
		}
		else if (request == COMTP_SHAKING_STARTED)
		{
			// Do nothing
		}
		else if (request == COMTP_SHAKING_STOPPED)
		{
			// Do nothing
		}
	}
}
