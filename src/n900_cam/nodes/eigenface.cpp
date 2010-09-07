/*
 * Eigenface, from example by Robin Hewitt,
 * adapted to fit into ROS
 * Careful, this is really, really ugly code, I'm not
 * fluent in C/C++, this is programming by coincidence...
*/


#include <sstream>

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"

#include "cv.h"
#include "cvaux.h"
#include "highgui.h"

#include "n900_cam/Whois.h"

//// Global variables (OpenCV
IplImage ** faceImgArr        = 0; // array of face images
CvMat    *  personNumTruthMat = 0; // array of person numbers
int nTrainFaces               = 100; // the number of training images
char* personName              = 0; // person name being "acquired"
int nCurrentFaces             = 0; // current face number during learning step
int nEigens                   = 0; // the number of eigenvalues
IplImage * pAvgTrainImg       = 0; // the average image
IplImage ** eigenVectArr      = 0; // eigenvectors
CvMat * eigenValMat           = 0; // eigenvalues
CvMat * projectedTrainFaceMat = 0; // projected training faces
CvMat * trainPersonNumMat     = 0; // the person numbers during training

//// Function prototypes (OpenCV)
void learn();
void recognize();
void doPCA();
void storeTrainingData(const char* filename = "facedata.xml");
int  loadTrainingData(CvMat ** pTrainPersonNumMat);
int*  findNearestNeighbor(float * projectedTestFace, int* results);
int  loadFaceImgArray(const char * filename);
void printUsage();
ros::Publisher whois_pub;


// Global variables (ROS)
sensor_msgs::CvBridge bridge_;
const int MODE_LEARN = 0;
const int MODE_RECOGNIZE = 1;
int currentMode = 1;

// Function prototypes (ROS)
void imageCallback(const sensor_msgs::ImageConstPtr& face);
void recognizeLive(IplImage* cv_image);
void acquire(IplImage* cv_image);


int main(int argc, char **argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line. For programmatic
     * remappings you can use a different version of init() which takes remappings
     * directly, but for most command-line programs, passing argc and argv is the easiest
     * way to do it.    The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "eigenface");

    if(argc >= 2)
    {
        if( !strcmp(argv[1], "acquire") )
        {
            personName = argv[2];
            ROS_INFO("argc: %d",argc);
            if(argc == 4)
                nTrainFaces = atoi(argv[3]);
            currentMode = MODE_LEARN;
            // init array containing training faces
            faceImgArr = (IplImage **)cvAlloc( nTrainFaces*sizeof(IplImage *) );
	        personNumTruthMat = cvCreateMat( 1, nTrainFaces, CV_32SC1 );
            ROS_INFO("Acquiring %d %s's faces...",nTrainFaces,personName);
        }
        else if( !strcmp(argv[1], "recognize") )
        {
	        // load the saved training data
            if( !loadTrainingData( &trainPersonNumMat ) ) return 1;
            currentMode = MODE_RECOGNIZE;
        }
        else if( !strcmp(argv[1], "learn") )
        {
            learn();
            return 0;
        }
        else if(!strcmp(argv[1], "test_recognize") )
        {
            recognize();
            return 0;
        }

    }
	else
	{
		printf("Unknown command: %s\n", argv[1]);
		printUsage();
        return 255;
	}

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("face_topic", 1000, imageCallback);
    whois_pub = n.advertise<n900_cam::Whois>("whois", 1000);

    ros::Rate loop_rate(10);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok())
    {

        ///**
        // * This is a message object. You stuff it with data, and then publish it.
        // */
        //std_msgs::String msg;

        //std::stringstream ss;
        //ss << "hello world " << count;
        //msg.data = ss.str();

        //ROS_INFO("%s", msg.data.c_str());

        ///**
        // * The publish() function is how you send messages. The parameter
        // * is the message object. The type of this object must agree with the type
        // * given as a template parameter to the advertise<>() call, as was done
        // * in the constructor above.
        // */
        //chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;

    }


    return 0;
}


//////////////////////////////////
// learn()
//
void learn()
{
	int i, offset;

	// load training data
	nTrainFaces = loadFaceImgArray("data/train.txt");
	if( nTrainFaces < 2 )
	{
		fprintf(stderr,
		        "Need 2 or more training faces\n"
		        "Input file contains only %d\n", nTrainFaces);
		return;
	}

	// do PCA on the training faces
	doPCA();

	// project the training images onto the PCA subspace
	projectedTrainFaceMat = cvCreateMat( nTrainFaces, nEigens, CV_32FC1 );
	offset = projectedTrainFaceMat->step / sizeof(float);
	for(i=0; i<nTrainFaces; i++)
	{
		//int offset = i * nEigens;
		cvEigenDecomposite(
			faceImgArr[i],
			nEigens,
			eigenVectArr,
			0, 0,
			pAvgTrainImg,
			//projectedTrainFaceMat->data.fl + i*nEigens);
			projectedTrainFaceMat->data.fl + i*offset);
	}

	// store the recognition data as an xml file
	storeTrainingData();
    ROS_INFO("Learned %d faces from face database",nTrainFaces);
}


//////////////////////////////////
// recognize()
//
void recognize()
{
	int i, nTestFaces  = 0;         // the number of test images
	float * projectedTestFace = 0;

	// load test images and ground truth for person number
	nTestFaces = loadFaceImgArray("data/test.txt");
	printf("%d test faces loaded\n", nTestFaces);

	// load the saved training data
	if( !loadTrainingData( &trainPersonNumMat ) ) return;

	// project the test images onto the PCA subspace
	projectedTestFace = (float *)cvAlloc( nEigens*sizeof(float) );
	for(i=0; i<nTestFaces; i++)
	{
		int iNearest, nearest, truth;

		// project the test image onto the PCA subspace
		cvEigenDecomposite(
			faceImgArr[i],
			nEigens,
			eigenVectArr,
			0, 0,
			pAvgTrainImg,
			projectedTestFace);

		//iNearest = findNearestNeighbor(projectedTestFace);
        int *res = (int*)malloc(2*sizeof(int));
        findNearestNeighbor(projectedTestFace,res);
        iNearest = res[0];
		truth    = personNumTruthMat->data.i[i];
		nearest  = trainPersonNumMat->data.i[iNearest];
        free(res);

		printf("nearest = %d, Truth = %d\n", nearest, truth);
	}
}


//////////////////////////////////
// loadTrainingData()
//
int loadTrainingData(CvMat ** pTrainPersonNumMat)
{
	CvFileStorage * fileStorage;
	int i;

	// create a file-storage interface
	fileStorage = cvOpenFileStorage( "facedata.xml", 0, CV_STORAGE_READ );
	if( !fileStorage )
	{
		fprintf(stderr, "Can't open facedata.xml\n");
		return 0;
	}


	nEigens = cvReadIntByName(fileStorage, 0, "nEigens", 0);
	nTrainFaces = cvReadIntByName(fileStorage, 0, "nTrainFaces", 0);
	*pTrainPersonNumMat = (CvMat *)cvReadByName(fileStorage, 0, "trainPersonNumMat", 0);
	eigenValMat  = (CvMat *)cvReadByName(fileStorage, 0, "eigenValMat", 0);
	projectedTrainFaceMat = (CvMat *)cvReadByName(fileStorage, 0, "projectedTrainFaceMat", 0);
	pAvgTrainImg = (IplImage *)cvReadByName(fileStorage, 0, "avgTrainImg", 0);
	eigenVectArr = (IplImage **)cvAlloc(nTrainFaces*sizeof(IplImage *));
	for(i=0; i<nEigens; i++)
	{
		char varname[200];
		sprintf( varname, "eigenVect_%d", i );
		eigenVectArr[i] = (IplImage *)cvReadByName(fileStorage, 0, varname, 0);
	}

	// release the file-storage interface
	cvReleaseFileStorage( &fileStorage );

	return 1;
}


//////////////////////////////////
// storeTrainingData()
//
void storeTrainingData(const char* filename)
{
	CvFileStorage * fileStorage;
	int i;

	// create a file-storage interface
	fileStorage = cvOpenFileStorage(filename, 0, CV_STORAGE_WRITE );

	// store all the data
	cvWriteInt( fileStorage, "nEigens", nEigens );
	cvWriteInt( fileStorage, "nTrainFaces", nTrainFaces );
	cvWrite(fileStorage, "trainPersonNumMat", personNumTruthMat, cvAttrList(0,0));
	cvWrite(fileStorage, "eigenValMat", eigenValMat, cvAttrList(0,0));
	cvWrite(fileStorage, "projectedTrainFaceMat", projectedTrainFaceMat, cvAttrList(0,0));
	cvWrite(fileStorage, "avgTrainImg", pAvgTrainImg, cvAttrList(0,0));
	for(i=0; i<nEigens; i++)
	{
		char varname[200];
		sprintf( varname, "eigenVect_%d", i );
		cvWrite(fileStorage, varname, eigenVectArr[i], cvAttrList(0,0));
	}

	// release the file-storage interface
	cvReleaseFileStorage( &fileStorage );
}


//////////////////////////////////
// findNearestNeighbor()
//
int* findNearestNeighbor(float * projectedTestFace,int* results)
{
	//double leastDistSq = 1e12;
	double leastDistSq = DBL_MAX;
	int i, iTrain, iNearest = 0;

	for(iTrain=0; iTrain<nTrainFaces; iTrain++)
	{
		double distSq=0;

		for(i=0; i<nEigens; i++)
		{
			float d_i =
				projectedTestFace[i] -
				projectedTrainFaceMat->data.fl[iTrain*nEigens + i];
			//distSq += d_i*d_i / eigenValMat->data.fl[i];  // Mahalanobis
			distSq += d_i*d_i; // Euclidean
		}

		if(distSq < leastDistSq)
		{
			leastDistSq = distSq;
			iNearest = iTrain;
            results[0] = iNearest;
            results[1] = int(distSq);
		}
	}

	return results;
}


//////////////////////////////////
// doPCA()
//
void doPCA()
{
	int i;
	CvTermCriteria calcLimit;
	CvSize faceImgSize;

	// set the number of eigenvalues to use
	nEigens = nTrainFaces-1;

	// allocate the eigenvector images
	faceImgSize.width  = faceImgArr[0]->width;
	faceImgSize.height = faceImgArr[0]->height;
	eigenVectArr = (IplImage**)cvAlloc(sizeof(IplImage*) * nEigens);
	for(i=0; i<nEigens; i++)
		eigenVectArr[i] = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);

	// allocate the eigenvalue array
	eigenValMat = cvCreateMat( 1, nEigens, CV_32FC1 );

	// allocate the averaged image
	pAvgTrainImg = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);

	// set the PCA termination criterion
	calcLimit = cvTermCriteria( CV_TERMCRIT_ITER, nEigens, 1);

	// compute average image, eigenvalues, and eigenvectors
	cvCalcEigenObjects(
		nTrainFaces,
		(void*)faceImgArr,
		(void*)eigenVectArr,
		CV_EIGOBJ_NO_CALLBACK,
		0,
		0,
		&calcLimit,
		pAvgTrainImg,
		eigenValMat->data.fl);

	cvNormalize(eigenValMat, eigenValMat, 1, 0, CV_L1, 0);
    
}


//////////////////////////////////
// loadFaceImgArray()
//
int loadFaceImgArray(const char * filename)
{
	FILE * imgListFile = 0;
	char imgFilename[512];
	int iFace, nFaces=0;


	// open the input file
	if( !(imgListFile = fopen(filename, "r")) )
	{
		fprintf(stderr, "Can\'t open file %s\n", filename);
		return 0;
	}

	// count the number of faces
	while( fgets(imgFilename, 512, imgListFile) ) ++nFaces;
	rewind(imgListFile);

	// allocate the face-image array and person number matrix
	faceImgArr        = (IplImage **)cvAlloc( nFaces*sizeof(IplImage *) );
	personNumTruthMat = cvCreateMat( 1, nFaces, CV_32SC1 );

	// store the face images in an array
	for(iFace=0; iFace<nFaces; iFace++)
	{
		// read person number and name of image file
		fscanf(imgListFile,
			"%d %s", personNumTruthMat->data.i+iFace, imgFilename);

		// load the face image
		faceImgArr[iFace] = cvLoadImage(imgFilename, CV_LOAD_IMAGE_GRAYSCALE);

		if( !faceImgArr[iFace] )
		{
			fprintf(stderr, "Can\'t load image from %s\n", imgFilename);
			return 0;
		}
	}

	fclose(imgListFile);

	return nFaces;
}


//////////////////////////////////
// printUsage()
//
void printUsage()
{
	ROS_INFO("Usage: eigenface <command>\nValid commands are\n     acquire <name>\n     recognize\n     learn");
}


void imageCallback(const sensor_msgs::ImageConstPtr& img)
{
    ROS_INFO("Got Face message");

    IplImage *cv_image = NULL;
    //sensor_msgs::ImageConstPtr image_ptr(face, &face->image);
    try
    {
        cv_image = bridge_.imgMsgToCv(img, "mono8");
    }
    catch (sensor_msgs::CvBridgeException error)
    {
        ROS_ERROR("error");
    }

    cvShowImage("Image window", cv_image);
    cvWaitKey(3);

    ROS_INFO("currentMode: %d",currentMode);

    if(currentMode == MODE_LEARN)
    {
        acquire(cv_image);
    }
    else if(currentMode == MODE_RECOGNIZE)
    {
        recognizeLive(cv_image);
    }
    else
    {
        ROS_INFO("No mode defined");
    }
}

void recognizeLive(IplImage* cv_image)
{
	float * projectedTestFace = 0;

	// project the test images onto the PCA subspace
	projectedTestFace = (float *)cvAlloc( nEigens*sizeof(float) );
	int iNearest, nearest, euclDist;

    // Grayscale only
    IplImage* scaledimg = NULL;

    scaledimg = cvCreateImage( cvSize(90,90), 8, 1 );
    cvResize(cv_image,scaledimg,CV_INTER_NN);

    //cvWaitKey(1000);

	// project the test image onto the PCA subspace
	cvEigenDecomposite(
		scaledimg,
		nEigens,
		eigenVectArr,
		0, 0,
		pAvgTrainImg,
		projectedTestFace);

    cvReleaseImage(&scaledimg);

	personNumTruthMat = cvCreateMat( 1, 1, CV_32SC1 );

	//iNearest = findNearestNeighbor(projectedTestFace);
    int* res = (int*)malloc(2*sizeof(int));
    findNearestNeighbor(projectedTestFace,res);
    iNearest = res[0];
    euclDist = res[1];
    // only keep closest faces
    if(res[1] < 10000000)
    {
        nearest  = trainPersonNumMat->data.i[iNearest];
        ROS_INFO("Live ! nearest = %d (%d)", nearest,euclDist);
        std::stringstream outputtmp;
        std::string output;
        outputtmp << nearest;
        output = outputtmp.str();
        cvShowImage(output.c_str(), cv_image);
        //cvWaitKey(3);
        n900_cam::Whois who;
        //std_msgs::String who;

        who.name = outputtmp.str();
        who.distance = euclDist;
        whois_pub.publish(who);

    }
    free(res);
    cvFree(&projectedTestFace);
    cvFree(&personNumTruthMat);

}

void acquire(IplImage* cv_image)
{
	int  offset;
    ROS_INFO("Live learning !");

    if(cv_image->width > 10)
    {
        std::stringstream outputtmp;
        std::string output;
        outputtmp << "data/" << personName << "/pix" << nCurrentFaces << ".jpg";
        output = outputtmp.str();
        // Scale image
        IplImage* scaledimg = cvCreateImage( cvSize(90,90), 8, 1 );
        cvResize(cv_image,scaledimg,CV_INTER_NN);
        cvSaveImage(output.c_str(),scaledimg);
		faceImgArr[nCurrentFaces] = cvLoadImage(output.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
		if( !faceImgArr[nCurrentFaces] )
            ROS_INFO("Troubles ahead...");
        nCurrentFaces++;
        ROS_INFO("Acquired %d/%d faces...",nCurrentFaces,nTrainFaces);
    }
    if(nCurrentFaces != nTrainFaces)
    {
        return;
    }

    ROS_INFO("Enough acquired faces");
    exit(0);
}

