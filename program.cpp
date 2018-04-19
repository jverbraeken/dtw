#pragma comment(lib, "Ws2_32.lib")
#include <winsock2.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <time.h>
#include "dtw.h"
#include "CircularBuffer.h"

#define BUFLEN 512
#define PORT 55056

static const unsigned char COMTP_SENSOR_DATA = 1;
static const unsigned char COMTP_SHAKING_STARTED = 2;
static const unsigned char COMTP_SHAKING_STOPPED = 3;

static const unsigned int ERROR_THRESHOLD = 7.5;

static const unsigned double MINIMUM_TIME_BETWEEN_GESTURES = 1.0;

static const unsigned int DATA_FREQUENCY = 25;

static const vector<int> DATA_BUFFER_SIZE = {
	DATA_FREQUENCY * 1,
	DATA_FREQUENCY * 2,
	DATA_FREQUENCY * 3,
	DATA_FREQUENCY * 5,
	DATA_FREQUENCY * 7,
	DATA_FREQUENCY * 10
};

unsigned int numGestures;
// per gesture per sample the duration category
vector<vector<int>*> preGestureDuration;
// per gesture per sample per dimension
vector<vector<vector<vector<double>*>*>*> preGestureValues;
// per resizement per gesture per dimension per sample
// vector<vector<vector<vector<float>*>*>*> resizedAveragedPreGestureValues;

// per dimension per timesequence length a CircularBuffer
vector<vector<CircularBuffer<double> *> *> * dataBuffer;

// per gesture per sample the detection time
vector<vector<time_t*>*> gestureDetectionTime;
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

vector<float>* resizeVector(int newSize, vector<float>* originalVector)
{
	double scalingFactor = double(originalVector->size()) / double(newSize);
	vector<float>* result = new vector<float>;
	for (int k = 0; k < newSize; k++)
	{
		double avg = 0;
		if (k > 0) { // just for efficiency
			double firstFactor = ceil(k * scalingFactor - 0.0001) - k * scalingFactor;
			if (abs(firstFactor) > 0.0001) {
				avg += firstFactor * (*originalVector)[floor(k * scalingFactor)];
			}
		}
		for (int l = ceil(k * scalingFactor - 0.0001); l < floor((k + 1) * scalingFactor); l++)
		{
			avg += (*originalVector)[l];
		}
		double lastFactor = (k + 1) * scalingFactor - floor((k + 1) * scalingFactor + 0.0001);
		if (abs(lastFactor) > 0.0001) {
			avg += lastFactor * (*originalVector)[floor((k + 1) * scalingFactor + 0.0001)];
		}
		avg /= scalingFactor;
		result->push_back(float(avg));
	}
	return result;
}

float addToAverage(float average, int size, float value)
{
	return (size * average + value) / (size + 1);
}



void checkDataBuffer() {
	time_t currentTime;
	time(&currentTime);
	for (int gesture = 0; gesture < numGestures; gesture++) {
		for (int sample = 0; sample < (*preGestureValues[gesture]).size(); sample++) {
			if ((*(gestureBounty[gesture]))[sample] > 0) {
				(*(gestureBounty[gesture]))[sample]--;
				continue;
			}
			vector<double> dataX = (*(*dataBuffer)[0])[(*(preGestureDuration[gesture]))[sample]]->getData();
			vector<double> dataY = (*(*dataBuffer)[1])[(*(preGestureDuration[gesture]))[sample]]->getData();
			vector<double> dataZ = (*(*dataBuffer)[2])[(*(preGestureDuration[gesture]))[sample]]->getData();
			vector<double> newDataX;
			vector<double> newDataY;
			if (currentTime - *(*(gestureDetectionTime[gesture]))[sample] > MINIMUM_TIME_BETWEEN_GESTURES) {
				double error = 0;
				double someBufferSize = (*(*dataBuffer)[0])[(*(preGestureDuration[gesture]))[sample]]->getData().getSize();
				double someBufferCapacity = (*(*dataBuffer)[0])[(*(preGestureDuration[gesture]))[sample]]->getSize();
				if (someBufferSize == someBufferCapacity) {
					double startAngle = atan2((*(*(*dataBuffer)[0])[(*(preGestureDuration[gesture]))[sample]])[0], (*(*(*dataBuffer)[1])[(*(preGestureDuration[gesture]))[sample]])[0]);
					double cs = cos(startAngle);
					double sn = sin(startAngle);
					for (int i = 0; i < dataX.size(); i++) {
						newDataX.push_back(dataX[i] * cs - dataY[i] * sn);
						newDataY.push_back(dataX[i] * sn + dataY[i] * cs);
					}
					LB_Improved filterX((*(*(*(preGestureValues[gesture]))[sample])[0]), 12);
					error += filterX.test(newDataX);
					LB_Improved filterY((*(*(*(preGestureValues[gesture]))[sample])[1]), 12);
					error += filterY.test(newDataY);
					LB_Improved filterZ((*(*(*(preGestureValues[gesture]))[sample])[2]), 12);
					error += filterZ.test(dataZ);
				} else {
					error += 999999;
				}
				printf("%f\n", error);
				if (error < ERROR_THRESHOLD) {
					printf("\n\nGESTURE MATCHED %d-%d\n\n", gesture, sample);
					time((*(gestureDetectionTime[gesture]))[sample]);
				} else {
					(*(gestureBounty[gesture]))[sample] = error < ERROR_THRESHOLD * 1.3 ? 1 : (int) (error - ERROR_THRESHOLD) / 2.5;
				}
			}
		}
	}
}



void useGRT() {
	preGestureValues = vector<vector<vector<vector<double>*>*>*>();

	cout << "Testing file \"walking_mobile.grt\"" << endl << endl;
	ifstream in;
	in.open("C:\\Users\\jverb\\Documents\\Git\\dtw-test\\walking_mobile.grt");
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
		preGestureDuration.push_back(new vector<int>());
		gestureDetectionTime.push_back(new vector<time_t*>());
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
	for (UINT x = 0; x < numTrainingExamples; x++) {
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
		for (int i = 0; i < sizeof(DATA_BUFFER_SIZE); i++) {
			if (duration / (1000 / DATA_FREQUENCY) < DATA_BUFFER_SIZE[i]) {
				(*(preGestureDuration.back())).push_back(i);
				break;
			}
		}

		in >> word;
		if (word != "TimeSeriesData:") {
			in.close();
			cout << "loadDatasetFromFile(std::string filename) - Failed to find TimeSeriesData!" << std::endl;
		}

		preGestureValues[classLabel]->push_back(new vector<vector<double>*>());

		for (int i = 0; i < 6; i++) {
			preGestureValues[classLabel]->back()->push_back(new vector<double>());
		}

		gestureDetectionTime[classLabel]->push_back(new time_t());
		time(gestureDetectionTime[classLabel]->back());

		gestureBounty[classLabel]->push_back(25);

		double startAngle;
		for (int i = 0; i < timeSeriesLength; i++) {
			float x, y, z, nx, ny;
			in >> x;
			in >> y;
			in >> z;
			x /= 100.f;
			y /= 100.f;
			z /= 100.f;
			if (i == 0) {
				startAngle = atan2(x, y);
			}
			double cs = cos(startAngle);
			double sn = sin(startAngle);
			nx = x * cs - y * sn;
			ny = x * sn + y * cs;
			(*preGestureValues[classLabel]->back())[0]->push_back(nx);
			(*preGestureValues[classLabel]->back())[1]->push_back(ny);
			(*preGestureValues[classLabel]->back())[2]->push_back(z);
			for (int j = 3; j < 6; j++) {
				float val;
				in >> val;
				(*preGestureValues[classLabel]->back())[j]->push_back(val / 100.f);
			}
		}
	}
	in.close();
}

int main() {
	cout << "UWF testing" << endl;

	dataBuffer = new vector<vector<CircularBuffer<double> *> *>();

	for (int i = 0; i < 6; i++) {
		vector<CircularBuffer<double>*>* vec = new vector<CircularBuffer<double>*>();
		for (int j = 0; j < DATA_BUFFER_SIZE.size(); j++) {
			vec->push_back(new CircularBuffer<double>(DATA_BUFFER_SIZE[j]));
		}
		dataBuffer->push_back(vec);
	}

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
			float xrot = getFloatFromBuffer(buf, 1);
			float yrot = getFloatFromBuffer(buf, 5);
			float zrot = getFloatFromBuffer(buf, 9);
			//long long rot_timestamp = getLongFromBuffer(buf, 13);

			float xacc = getFloatFromBuffer(buf, 21);
			float yacc = getFloatFromBuffer(buf, 25);
			float zacc = getFloatFromBuffer(buf, 29);
			//long long acc_timestamp = getLongFromBuffer(buf, 33);

			vector<float> data = vector<float>();
			data.push_back(xrot);
			data.push_back(yrot);
			data.push_back(zrot);
			data.push_back(xacc);
			data.push_back(yacc);
			data.push_back(zacc);

			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < DATA_BUFFER_SIZE.size(); j++) {
					(*(*dataBuffer)[i])[j]->push_back(data[i]);
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
