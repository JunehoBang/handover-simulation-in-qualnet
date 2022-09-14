// Copyright (c) 2001-2013, SCALABLE Network Technologies, Inc.  All Rights Reserved.
//                          600 Corporate Pointe
//                          Suite 1200
//                          Culver City, CA 90230
//                          info@scalable-networks.com
//
// This source code is licensed, not sold, and is subject to a written
// license agreement.  Among other things, no portion of this source
// code may be copied, transmitted, disclosed, displayed, distributed,
// translated, used as the basis for a derivative work, or used, in
// whole or in part, for any program or purpose other than its intended
// use in compliance with the license agreement as part of the QualNet
// software.  This source code and certain of the algorithms contained
// within it are confidential trade secrets of Scalable Network
// Technologies, Inc. and may not be used as the basis for any other
// software, hardware, product or service.


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#include <cmath>   

#include "api.h"
#include "partition.h"
#include "network_ip.h"

#include "phy_lte.h"
#include "phy_lte_establishment.h"
#include "layer2_lte_mac.h"
#include "layer2_lte_pdcp.h"
#include "layer2_lte_rlc.h"
#include "layer3_lte_filtering.h"
#include "layer3_lte.h"
#include "epc_lte.h"
#include "math.h"

#include <iostream>
#include <string>
#include <fstream>

#ifdef LTE_LIB_LOG
#include "log_lte.h"


#include "epc_lte_app.h"

#define DEBUG              0




//bang
bool operator<(const TrainingDatum& lhs, const TrainingDatum& rhs)
{
	return lhs.measuredTime < rhs.measuredTime;
}
bool operator==(const TrainingDatum& lhs, const TrainingDatum& rhs)
{
	return lhs.measuredTime == rhs.measuredTime;
}
//bool operator>(const TrainingDatum& lhs, const TrainingDatum& rhs)
//{
//	return lhs.measuredTime > rhs.measuredTime;
//}

bool operator<(const ExecutingHandover& lhs, const ExecutingHandover& rhs)
{
	return lhs.ueRnti < rhs.ueRnti; 
}
bool operator==(const ExecutingHandover& lhs, const ExecutingHandover& rhs)
{
	return lhs.ueRnti == rhs.ueRnti;
}
bool operator>(const ExecutingHandover& lhs, const ExecutingHandover& rhs)
{
	return lhs.ueRnti.nodeId > rhs.ueRnti.nodeId;
}

void Layer3LteInitialize(
    Node* node, int interfaceIndex, const NodeInput* nodeInput)
{
    Layer3DataLte* layer3Data =
        LteLayer2GetLayer3DataLte(node, interfaceIndex);
	BOOL wasFound = FALSE;

	NodeAddress interfaceAddress =
		MAPPING_GetInterfaceAddrForNodeIdAndIntfId(
		node, node->nodeId, interfaceIndex);

    // init parameter
    layer3Data->layer3LteState = LAYER3_LTE_POWER_OFF;
    layer3Data->connectionInfoMap.clear();
    layer3Data->waitRrcConnectedTime = 0;
    layer3Data->waitRrcConnectedReconfTime = 0;
    layer3Data->ignoreHoDecisionTime = 0;

    // read config
    Layer3LteInitConfigurableParameters(node, interfaceIndex, nodeInput);

	//Bang
	
	layer3Data->handoverCandidateDB=new HandoverCandidateDB;
	layer3Data->trainingDataDB=new TrainingDataDB;
	layer3Data->executingHandoverDB = new ExecutingHandoverDB;
	layer3Data->handoverComplete = false;
	layer3Data->handoverExecutionUnderway = false;
	layer3Data->lastRsrpFromSrc = -9999999;
	layer3Data->firstRsrpFromDst = -9999999;

	layer3Data->minRsrpFromConnectedEnb = 9999999;

	char hoDecisionAlgorithm[30];
	IO_ReadString(node->nodeId, interfaceAddress,nodeInput,"RRC-LTE-HANDOVER-DECISION-ALGORITHM",&wasFound, hoDecisionAlgorithm);
	clocktype tempT;
	double tempD;
	IO_ReadDouble(node->nodeId,interfaceAddress,nodeInput,"RRC-LTE-CELL-BOUNDARY-BEGIN",&wasFound,&tempD);
	node->cellBoundaryBegin = tempD;
	IO_ReadDouble(node->nodeId,interfaceAddress,nodeInput,"RRC-LTE-CELL-BOUNDARY-END",&wasFound,&tempD);
	node->cellBoundaryEnd = tempD;
	node->firstPacketRx = 0;
	node->lastPacketRx = 0;
	node->transmittedInCellBoundary = new PacketIdSet;
	node->receivedInCellBoundary = new PacketIdSet;
	node->receivedWithMark = new PacketIdSet;

	if(strcmp(hoDecisionAlgorithm,"A3-EVENT-BASED")==0)
	{
		layer3Data->hoDecisionAlgorithm=A3_EVENT_BASED_HO;
		IO_ReadDouble(node->nodeId,interfaceAddress,nodeInput,"RRC-LTE-MEAS-EVENT-A3-RSRP-HYS-FOR-ENB",&wasFound,&tempD);
		IO_ReadTime(node->nodeId, interfaceAddress,nodeInput, "RRC-LTE-MEAS-TIME-TO-TRIGGER-FOR-A3-ENB",&wasFound,&tempT);
		layer3Data->timeToTrigger=tempT;
		layer3Data->hysteresis=tempD;
	}
	else if(strcmp(hoDecisionAlgorithm,"PREDICTION-BASED")==0)
	{
		
		layer3Data->hoDecisionAlgorithm=PREDICTION_BASED_HO;
		double degree;
		clocktype predictionTolerance;
		double tolerance;
				
		if(node->partitionData->ep ==NULL)
		{
			IO_ReadDouble(node->nodeId, interfaceAddress,nodeInput, "RRC-LTE-MEAS-PREDICTION-MODEL-DEGREE",&wasFound, &degree);
			ERROR_Assert(wasFound,"Degree information is required\n");
			
			IO_ReadTime(node->nodeId, interfaceAddress,nodeInput, "RRC-LTE-MEAS-PREDICTION-TOLERANCE",&wasFound, &predictionTolerance);
			ERROR_Assert(wasFound,"Tolerance information is required\n");
			tolerance = (double) predictionTolerance / SECOND;
		
			if (!(node->partitionData->ep = engOpen(""))) {
				ERROR_Assert(FALSE,"Unable to open MATLAB Engine\n");
			}

			Engine* ep = node->partitionData->ep;

			mxArray *C = NULL;
			C = mxCreateDoubleMatrix(1,1,mxREAL);
			engEvalString(ep, "clear");
			engEvalString(ep, "addpath('C:/Users/bang/Documents/MATLAB/BLR')");  //Add path
			
			memcpy((void*)mxGetPr(C),(void*)&degree,sizeof(double));			//initialize degree and tolerance
			engPutVariable(ep,"degree",C);

			memcpy((void*)mxGetPr(C),(void*)&tolerance,sizeof(double));			//Do not need to be stored in C++
			engPutVariable(ep,"tolerance",C);

			engEvalString(ep, "[mu, sig, alpha, beta] = trainMybrm(degree)");
			mxDestroyArray(C);

			//read degree from the file
			//read tolerance from the file
			//set the variable in the matlab and train the regression model
						
		}
		double decisionThreshold;
		IO_ReadDouble(node->nodeId, interfaceAddress,nodeInput, "RRC-LTE-MEAS-PREDICTION-DECISION-THRESHOLD",&wasFound, &decisionThreshold);
		if(!wasFound){
			//halt the simulation
		}
		layer3Data->decisionThreshold = decisionThreshold;
		//read decision thereshold from the file and save in ltelayer3
	}
	else if(strcmp(hoDecisionAlgorithm,"H2-EVENT-BASED")==0)
	{
		layer3Data->hoDecisionAlgorithm=H2_EVENT_BASED_HO;
		layer3Data->H2Trigger = FALSE;
		layer3Data->triggerDuration = 0;
		IO_ReadDouble(node->nodeId,interfaceAddress,nodeInput,"RRC-LTE-MEAS-EVENT-H2-RSRP-HYS-FOR-ENB",&wasFound,&tempD);
		layer3Data->hysteresis=tempD;
		IO_ReadTime(node->nodeId, interfaceAddress,nodeInput, "RRC-LTE-MEAS-TIME-TO-TRIGGER-FOR-H2-ENB",&wasFound,&tempT);
		layer3Data->timeToTrigger=tempT;
		IO_ReadTime(node->nodeId, interfaceAddress,nodeInput, "RRC-LTE-MEAS-REPORT-INTERVAL",&wasFound,&tempT);
		layer3Data->reportTxInterval=tempT;
		IO_ReadDouble(node->nodeId,interfaceAddress,nodeInput,"RRC-LTE-MEAS-EVENT-H2-TRIGGER-THRESHOLD",&wasFound,&tempD);
		layer3Data->triggerThreshold=tempD;
	}
    // init statistics
    layer3Data->statData.averageRetryRrcConnectionEstablishment = 0;
    layer3Data->statData.averageTimeOfRrcConnectionEstablishment = 0;
    layer3Data->statData.numRrcConnectionEstablishment = 0;
    layer3Data->establishmentStatData.lastStartTime = 0;
    layer3Data->establishmentStatData.numPowerOn = 0;
    layer3Data->establishmentStatData.numPowerOff = 0;

    // Power ON
    Layer3LtePowerOn(node, interfaceIndex);

#ifdef LTE_LIB_USE_POWER_TIMER
    {
        int num = 0;

        clocktype retTime = 0;
        NodeAddress interfaceAddress =
            MAPPING_GetInterfaceAddrForNodeIdAndIntfId(
                node, node->nodeId, interfaceIndex);
        IO_ReadInt(node->nodeId,
            interfaceAddress,
            nodeInput,
            RRC_LTE_STRING_NUM_POWER_ON,
            &wasFound,
            &num);
        if (wasFound == TRUE)
        {
            if (num > 0)
            {
                for (int i = 0; i < num; i++)
                {
                    IO_ReadTimeInstance(node->nodeId,
                        interfaceAddress,
                        nodeInput,
                        RRC_LTE_STRING_POWER_ON_TIME,
                        i,
                        (i == 0),
                        &wasFound,
                        &retTime);
                    if (wasFound == TRUE)
                    {
                        Message* onTimer = MESSAGE_Alloc(
                            node, MAC_LAYER, MAC_PROTOCOL_LTE,
                            MSG_RRC_LTE_WaitPowerOnTimerExpired);
                        MESSAGE_SetInstanceId(onTimer, interfaceIndex);
                        MESSAGE_Send(node, onTimer, retTime);
                    }
                }
            }
        }

        IO_ReadInt(node->nodeId,
            interfaceAddress,
            nodeInput,
            RRC_LTE_STRING_NUM_POWER_OFF,
            &wasFound,
            &num);
        if (wasFound == TRUE)
        {
            if (num > 0)
            {
                for (int i = 0; i < num; i++)
                {
                    IO_ReadTimeInstance(node->nodeId,
                        interfaceAddress,
                        nodeInput,
                        RRC_LTE_STRING_POWER_OFF_TIME,
                        i,
                        (i == 0),
                        &wasFound,
                        &retTime);
                    if (wasFound == TRUE)
                    {
                        Message* offTimer = MESSAGE_Alloc(
                            node, MAC_LAYER, MAC_PROTOCOL_LTE,
                            MSG_RRC_LTE_WaitPowerOffTimerExpired);
                        MESSAGE_SetInstanceId(offTimer, interfaceIndex);
                        MESSAGE_Send(node, offTimer, retTime);
                    }
                }
            }
        }
    }
#endif // LTE_LIB_USE_POWER_TIMER
    // setup measurement configration
    Layer3LteSetupMeasConfig(node, interfaceIndex, nodeInput);
}

// /**
// FUNCTION   :: layer3LteFinalize
// LAYER      :: RRC
// PURPOSE    :: Finalize RRC
// PARAMETERS ::
// + node           : Node*             : Pointer to node.
// + interfaceIndex : int               : Interface Index
// RETURN     :: void : NULL
// **/
void Layer3LteFinalize(
    Node* node, int interfaceIndex)
{
    Layer3DataLte* layer3Data =
        LteLayer2GetLayer3DataLte(node, interfaceIndex);
	FILE* fp;
    Layer3LtePrintStat(node, interfaceIndex, layer3Data->statData);

	LteStationType stationType =
		LteLayer2GetStationType(node, interfaceIndex);
	if (stationType == LTE_STATION_TYPE_UE &&layer3Data->hoDecisionAlgorithm == A3_EVENT_BASED_HO) //training file은 A3의 경우에만 출력됨
	{

		TrainingDataDB* trainingDataDB = layer3Data->trainingDataDB;
		double minRatio=100000;
		clocktype minTime;
		TrainingDataDB::iterator it;
		TrainingDataDB::iterator minit=trainingDataDB->end();
		for(it=trainingDataDB->begin();it!=trainingDataDB->end();it++)
		{

			if(minRatio > it->Log_RsrpRatio && it->Log_RsrpRatio > 0  && it->Log_Speed > -500 && it->Log_RsrpRatio < 0.3) 
			{

				minRatio = it->Log_RsrpRatio;
				minTime = it->measuredTime;
			}
		}
		//ofstream ofp;
		FILE* ofp;
		char filename[50];
		char nodenum[4];
		double waiting;
		itoa(node->nodeId,nodenum,10);
		strcpy(filename,"training");
		//strcat(filename,nodenum);
		strcat(filename,".txt");
		ofp = fopen(filename,"a");
		//ofp.open(filename);
		for(it=trainingDataDB->begin();it!=trainingDataDB->end();it++)
		{
			/*if(minTime == it->measuredTime)
			break;
			if (it->Log_RsrpRatio > 0 )
			{*/
			if(minTime != it->measuredTime && it->Log_RsrpRatio > 0 && it->Log_Speed > -500)
			{
				waiting = double((minTime - (it->measuredTime)))/double(SECOND);
				fprintf(ofp,"%lf %lf %lf\n",it->Log_Speed, it->Log_RsrpRatio, (double)log(waiting));
			}
		}
		
		fclose(ofp);
		//print training data
		//get minimum rsrp received
	}

	if(stationType == LTE_STATION_TYPE_UE)
	{
		double pdrInCellBoundary;
		double goodputInCellBoundary;
		double boundaryDuration = double(double(node->lastPacketRx - node->firstPacketRx)/SECOND);
		if(node->receivedWithMark->size() != 0)
		{
			double receivedWithMark = (double)node->receivedWithMark->size();
			double sent = (double)node->transmittedInCellBoundary->size();
			double receivedInCellBoundary=(double)node->receivedInCellBoundary->size();
			pdrInCellBoundary= receivedWithMark / sent;
			goodputInCellBoundary = receivedInCellBoundary / boundaryDuration;

			fp = fopen("performance.txt","a");
			fprintf(fp,"%lf %lf\n",pdrInCellBoundary,goodputInCellBoundary);
			fclose(fp);

			fp = fopen("pdr.txt","a");
			fprintf(fp,"%lf\n",pdrInCellBoundary);
			fclose(fp);

			fp = fopen("goodput.txt","a");
			fprintf(fp,"%lf\n",goodputInCellBoundary);
			fclose(fp);

			fp = fopen("minrsrp.txt","a");
			fprintf(fp,"%lf\n",layer3Data->minRsrpFromConnectedEnb);
			fclose(fp);
		}

		fp = fopen("RSRPgap.txt","a");
		fprintf(fp,"%lf\n",abs(layer3Data->lastRsrpFromSrc - layer3Data->firstRsrpFromDst));
		fclose(fp);
	}

	
	if(node->partitionData->ep !=NULL)
	{
		engClose(node->partitionData->ep);
		node->partitionData->ep = NULL;
	}

	delete layer3Data->handoverCandidateDB;
	delete layer3Data->trainingDataDB;
	delete layer3Data->executingHandoverDB;
	delete node->transmittedInCellBoundary;
	delete node->receivedInCellBoundary;
	delete node->receivedWithMark;

    layer3Data->connectionInfoMap.clear();
    delete layer3Data;
}



// /**
// FUNCTION   :: Layer3LteIFHPNotifyMeasurementReportReceived
// LAYER      :: RRC
// PURPOSE    :: IF for PHY to notify Measurement Repot Received to RRC
// PARAMETERS ::
// + node             : Node*             : Pointer to node.
// + interfaceIndex   : int               : Interface index
// + ueRnti           : const LteRnti&    : the source of the report
// + measurementReport: std::list<MeasurementReport>*: report
// RETURN     :: void : NULL
// **/
void Layer3LteIFHPNotifyMeasurementReportReceived(
                    Node *node,
                    UInt32 interfaceIndex,
                    const LteRnti& ueRnti,
                    std::list<MeasurementReport>* measurementReport)
{
    // station type guard
    LTE_LIB_STATION_TYPE_GUARD(node, interfaceIndex,
        LTE_STATION_TYPE_ENB, LTE_STRING_STATION_TYPE_ENB);

#ifdef LTE_LIB_LOG
    {
        lte::LteLog::InfoFormat(node, interfaceIndex,
            LTE_STRING_LAYER_TYPE_RRC,
            LAYER3_LTE_MEAS_CAT_HANDOVER","
            LTE_STRING_FORMAT_RNTI","
            "[received measurement reports]",
            ueRnti.nodeId, ueRnti.interfaceIndex);
    }
#endif

    // discard messages from UE which doesn't connect to this eNB
    LteConnectionInfo* connInfo =
        Layer3LteGetConnectionInfo(node, interfaceIndex, ueRnti);
    if (connInfo == NULL ||
        connInfo->state != LAYER3_LTE_CONNECTION_CONNECTED)
    {
#ifdef LTE_LIB_LOG
        {
            lte::LteLog::InfoFormat(node, interfaceIndex,
                LTE_STRING_LAYER_TYPE_RRC,
                LAYER3_LTE_MEAS_CAT_HANDOVER","
                LTE_STRING_FORMAT_RNTI","
                "[discard received measurement reports]",
                ueRnti.nodeId, ueRnti.interfaceIndex);
        }
#endif 
        return;
    }

    // discard messages from UE which hasn't finished previous H.O.
    BOOL receivedEndMarker = Layer3LteGetTimerForRrc(
        node, interfaceIndex, ueRnti, MSG_MAC_LTE_X2_WaitEndMarker)
        == NULL;
    BOOL receivedPathSwitchRequestAck = Layer3LteGetTimerForRrc(
        node, interfaceIndex, ueRnti, MSG_MAC_LTE_S1_WaitPathSwitchReqAck)
        == NULL;
    if (!receivedEndMarker || !receivedPathSwitchRequestAck)
    {
#ifdef LTE_LIB_LOG
        {
            lte::LteLog::InfoFormat(node, interfaceIndex,
                LTE_STRING_LAYER_TYPE_RRC,
                LAYER3_LTE_MEAS_CAT_HANDOVER","
                LTE_STRING_FORMAT_RNTI","
                "[discard received measurement reports because"
                " the UE hasn't finished previous H.O. yet.]",
                ueRnti.nodeId, ueRnti.interfaceIndex);
        }
#endif
        return;
    }

    // HO decision
    LteRnti targetRnti = LTE_INVALID_RNTI;

	Layer3DataLte* layer3Data =
		LteLayer2GetLayer3DataLte(node, interfaceIndex);
	switch(layer3Data->hoDecisionAlgorithm)
	{
		case A3_EVENT_BASED_HO:
		{
			Layer3LteHandleMeasurementReportForA3(node,interfaceIndex,ueRnti,
												  measurementReport);
			 
			break;
		}
		case PREDICTION_BASED_HO:
		{
			Layer3LteHandleMeasurementReportForPredictionBased(node, interfaceIndex, ueRnti, measurementReport);
			break;
		}
		case H2_EVENT_BASED_HO:
		{
			Layer3LteHandleMeasurementReportForH2(node,interfaceIndex,ueRnti,
												  measurementReport);
			break;
		}
		
	}
	


#include <cstdlib>
void Layer3LteHandleMeasurementReportForA3(Node* node, UInt32 interfaceIndex,
											const LteRnti& ueRnti,
											std::list<MeasurementReport>* measurementReport)
{
	
	Engine* ep = node->partitionData->ep;

	Layer3DataLte* layer3Data =
		LteLayer2GetLayer3DataLte(node, interfaceIndex);
	clocktype measuredTime;
	double speed;
	LteRnti targetRnti;
	double maxServRSRP;
	double maxNeighRSRP;
	Layer3LteSummarizeHandoverRelatedInfo(node, ueRnti, measurementReport, &targetRnti,&maxServRSRP,&maxNeighRSRP,&speed, &measuredTime);
	if(node->nodeId == 3)
		return;

	Layer2DataLte* layer2 = LteLayer2GetLayer2DataLte(node, interfaceIndex);
	
	//Bang
	Node* ue = MAPPING_GetNodePtrFromHash(
		node->partitionData->nodeIdHash,
		ueRnti.nodeId);
	Layer3DataLte* layer3Data_UE =
		LteLayer2GetLayer3DataLte(ue, 0);
	TrainingDatum newDatum;
	newDatum.measuredTime=measuredTime;
	newDatum.Log_Speed =log(speed);
	newDatum.handoverComplete=layer3Data_UE->handoverComplete;
	bool infFlag = (maxServRSRP==0||maxNeighRSRP==0);
	
	//measurement report를 받은 주체가 handover target eNB이다. 
	//if(layer3Data_UE->handoverComplete)  
	if(node->nodeId==3)
	{
		newDatum.Log_RsrpRatio = log(NON_DB(maxNeighRSRP)/NON_DB(maxServRSRP));
		newDatum.destENB=layer2->myRnti;
		newDatum.srcENB=targetRnti;

	}
	else
	{
		newDatum.Log_RsrpRatio = log(NON_DB(maxServRSRP)/NON_DB(maxNeighRSRP));
		newDatum.destENB=targetRnti;
		newDatum.srcENB=layer2->myRnti;
	}
	if(!infFlag)
		layer3Data_UE->trainingDataDB->insert(newDatum);
		
	ExecutingHandoverDB::iterator eit;
	ExecutingHandoverDB *executingHandoverDB = layer3Data->executingHandoverDB;
	for(eit=executingHandoverDB->begin();eit!=executingHandoverDB->end();eit++)
		if(eit->ueRnti==ueRnti)
			break;

	BOOL handoverIsUnderway= (eit != executingHandoverDB->end());

	if(handoverIsUnderway)
	{
		return;
	}
	
	HandoverCandidateDB* handoverCandidateDB=layer3Data->handoverCandidateDB;
	HandoverCandidateDB::iterator it;
	
	it=handoverCandidateDB->find(ueRnti);
	
	bool targetIsBetter=false;
	double hysteresis=layer3Data->hysteresis;
	if(maxServRSRP + hysteresis < maxNeighRSRP)
		targetIsBetter=true;
	
	if(it==handoverCandidateDB->end())
	{
		if(targetIsBetter)
		{
			handoverCandidateDB->insert(HandoverCandidate(ueRnti,targetRnti));
			//turn on the timer
			Layer3LteSetTimerForRrc(node, interfaceIndex, ueRnti,
				MSG_RRC_LTE_Handover_Waiting_Timer_Expired,layer3Data->timeToTrigger);
		}
		else
			return;
	}

	else 
	{
		if(targetIsBetter)
			return;
		else
		{
			handoverCandidateDB->erase(it);
			Layer3LteCancelTimerForRrc(node, interfaceIndex, ueRnti, MSG_RRC_LTE_Handover_Waiting_Timer_Expired);
		}

	}
}

void Layer3LteHandleMeasurementReportForPredictionBased(Node* node, UInt32 interfaceIndex,
	const LteRnti& ueRnti,std::list<MeasurementReport>* measurementReport)
{

	clocktype measuredTime;
	clocktype timeToCrossing;
	double speed;
	LteRnti targetRnti;
	double maxServRSRP;
	double maxNeighRSRP;
	Layer3LteSummarizeHandoverRelatedInfo(node, ueRnti, measurementReport, &targetRnti,&maxServRSRP,&maxNeighRSRP,&speed, &measuredTime);
	if(node->nodeId == 3)
		return;

	HandoverCandidateDB* handoverCandidateDB;
	HandoverCandidateDB::iterator it;
	Engine* ep = node->partitionData->ep;
	Layer3DataLte* layer3Data =
		LteLayer2GetLayer3DataLte(node, interfaceIndex);

	handoverCandidateDB = layer3Data->handoverCandidateDB;
	it = handoverCandidateDB->find(ueRnti);
	if(it != handoverCandidateDB->end())
		return;

	

	bool infFlag = (maxServRSRP==0||maxNeighRSRP==0);

	double x[2];
	x[0]=log(speed);
	x[1]=log(NON_DB(maxServRSRP)/NON_DB(maxNeighRSRP));

	if(x[1]<0)
	{
		timeToCrossing = 1*MICRO_SECOND;
		Layer3LteSetTimerForRrc(node, interfaceIndex, ueRnti,
			MSG_RRC_LTE_Handover_Waiting_Timer_Expired, timeToCrossing);
		handoverCandidateDB->insert(HandoverCandidate(ueRnti,targetRnti));
		return;
	}
	
	
	double decisionThreshold;
	decisionThreshold= layer3Data->decisionThreshold;

	double exp_mode;
	double intervalProb;

	mxArray *T = NULL;
	mxArray *D = NULL;

	mxArray *C = NULL;
	mxArray *X = NULL;

	C = mxCreateDoubleMatrix(1,1,mxREAL);
	X = mxCreateDoubleMatrix(1,2,mxREAL);

	memcpy((void*)mxGetPr(X),(void*)&x,sizeof(x));
	engPutVariable(ep,"x",X);

	engEvalString(ep, "[exp_mode, intervalProb] = crossingtimeprediction(mu, sig, alpha, beta, x, degree, tolerance)");	

	C = engGetVariable(ep,"exp_mode");
	memcpy((void*)&exp_mode,(void*)mxGetPr(C),sizeof(exp_mode));

	C = engGetVariable(ep,"intervalProb");
	memcpy((void*)&intervalProb,(void*)mxGetPr(C),sizeof(intervalProb));

	if(intervalProb > decisionThreshold)
	{
		clocktype predictedHandoverInit = measuredTime + (clocktype)(SECOND*exp_mode)+7*MILLI_SECOND;
		
		if(node->getNodeTime()> predictedHandoverInit)
			timeToCrossing = 1*MICRO_SECOND;
		else
			timeToCrossing = predictedHandoverInit-node->getNodeTime();
		Layer3LteSetTimerForRrc(node, interfaceIndex, ueRnti,
							MSG_RRC_LTE_Handover_Waiting_Timer_Expired, timeToCrossing);
		handoverCandidateDB->insert(HandoverCandidate(ueRnti,targetRnti));
	}
	mxDestroyArray(C);
	mxDestroyArray(X);

}

void Layer3LteHandleMeasurementReportForH2(Node* node, UInt32 interfaceIndex,
											const LteRnti& ueRnti,
											std::list<MeasurementReport>* measurementReport)
{
	clocktype measuredTime;
	clocktype timeToCrossing;
	double speed;
	LteRnti targetRnti;
	double maxServRSRP;
	double maxNeighRSRP;
	Layer3LteSummarizeHandoverRelatedInfo(node, ueRnti, measurementReport, &targetRnti,&maxServRSRP,&maxNeighRSRP,&speed, &measuredTime);

	if(node->nodeId == 3)
		return;
	HandoverCandidateDB* handoverCandidateDB;
	HandoverCandidateDB::iterator it;
	Engine* ep = node->partitionData->ep;
	Layer3DataLte* layer3Data =
		LteLayer2GetLayer3DataLte(node, interfaceIndex);

	handoverCandidateDB = layer3Data->handoverCandidateDB;
	it = handoverCandidateDB->find(ueRnti);
	if(it != handoverCandidateDB->end())
		return;

	

	bool infFlag = (maxServRSRP==0||maxNeighRSRP==0);
	bool performHO=false;

	Node* ue = MAPPING_GetNodePtrFromHash(
		node->partitionData->nodeIdHash,
		ueRnti.nodeId);
	Layer3DataLte* layer3Data_UE =
		LteLayer2GetLayer3DataLte(ue, 0);
	                                          //Ofn, Ocn   R_trig
	if(!layer3Data_UE->H2Trigger && maxNeighRSRP+3+3>(layer3Data_UE->triggerThreshold)+layer3Data_UE->hysteresis)
	{
		layer3Data_UE->H2Trigger = TRUE;
		layer3Data_UE->triggerDuration = layer3Data_UE->triggerDuration + layer3Data_UE->reportTxInterval;
	}
	else if(layer3Data_UE->H2Trigger)
	{                 //Ofn, Ocn                 
		if(maxNeighRSRP+3+3+(layer3Data_UE->hysteresis)<layer3Data->triggerThreshold)
			layer3Data_UE->H2Trigger = FALSE;
		                    //Ofn, Ocn      Ofs, Ocs, Off
		else if(maxNeighRSRP+3+3>maxServRSRP+3+3+3+layer3Data_UE->hysteresis)
		{
			performHO = true;
		}
		else
		{
			layer3Data_UE->triggerDuration = layer3Data_UE->triggerDuration + layer3Data_UE->reportTxInterval;
			if(layer3Data_UE->triggerDuration>layer3Data_UE->hysteresis)
				performHO = true;
		}

	}

	if(performHO)
	{
		timeToCrossing = MICRO_SECOND;
		Layer3LteSetTimerForRrc(node, interfaceIndex, ueRnti,
			MSG_RRC_LTE_Handover_Waiting_Timer_Expired, timeToCrossing);
		handoverCandidateDB->insert(HandoverCandidate(ueRnti,targetRnti));
	}
	
}

void Layer3LteSummarizeHandoverRelatedInfo(Node* node, const LteRnti& ueRnti, std::list<MeasurementReport>* measurementReport,
	LteRnti* targetRnti, double* maxServRSRP, double* maxNeighRSRP, double* speed,
	clocktype* measuredTime)
{
	*targetRnti =LTE_INVALID_RNTI;
	*maxServRSRP = measurementReport->begin()
		->measResults.measResultServCell.rsrpResult;
	for (std::list<MeasurementReport>::iterator it =					//Bang: This is array of measurement reports
		measurementReport->begin(); it != measurementReport->end(); ++it)
	{
		*maxServRSRP =
			max(it->measResults.measResultServCell.rsrpResult, *maxServRSRP);
		*measuredTime=it->measuredTime;
		*speed = it->speed;
	}

	Node* ue = MAPPING_GetNodePtrFromHash(
		node->partitionData->nodeIdHash,
		ueRnti.nodeId);
	Layer3DataLte* layer3Data_UE =
		LteLayer2GetLayer3DataLte(ue, 0);
	if(layer3Data_UE->minRsrpFromConnectedEnb > *maxServRSRP)
		layer3Data_UE->minRsrpFromConnectedEnb = *maxServRSRP;
	
	*maxNeighRSRP=-5000;
	for (std::list<MeasurementReport>::iterator it =
		measurementReport->begin(); it != measurementReport->end(); ++it)
	{
		ListMeasResultInfo& results = it->measResults.measResultNeighCells;
		for (ListMeasResultInfo::iterator it_result = results.begin();
			it_result != results.end(); ++it_result)
		{
			// skip nodes over our EPC subnet
			if (!EpcLteAppIsNodeOnTheSameEpcSubnet(
				node, it_result->rnti.nodeId))
			{
				continue;
			}

			// update max
			if (it_result->rsrpResult > *maxNeighRSRP)
			{
				*maxNeighRSRP = it_result->rsrpResult;
				*targetRnti = it_result->rnti;
			}
		}

	}
}
void Layer3LteHandoverWaitingTimerExpired(Node* node, UInt32 interfaceIndex, Message* msg)
{
	HandoverCandidateDB::iterator it;
	Layer3DataLte* layer3Data =
		LteLayer2GetLayer3DataLte(node, interfaceIndex);	
	HandoverCandidateDB* handoverCandidateDB= layer3Data->handoverCandidateDB;
	LteRnti* ueRnti=(LteRnti*)MESSAGE_ReturnInfo(msg);
	Layer2DataLte* layer2 = LteLayer2GetLayer2DataLte(node, interfaceIndex);
	HandoverParticipator handoverParticipator;
	// find handoverCandidate in the DB
	
	it=handoverCandidateDB->find(*ueRnti);
	if(it==handoverCandidateDB->end())
		return;
	
	handoverParticipator.ueRnti=*ueRnti;
	handoverParticipator.srcEnbRnti=layer2->myRnti;
	handoverParticipator.tgtEnbRnti=it->second;
	 
	
	//delete it in the DB
	handoverCandidateDB->erase(it);

	//send path switch message
	EpcLteAppSend_HandoverRequest(node, interfaceIndex, handoverParticipator);
	ExecutingHandover executingHandover;
	executingHandover.ueRnti = *ueRnti;
	executingHandover.initiation = node->getNodeTime();
	layer3Data->executingHandoverDB->insert(executingHandover);

	//handover ack message timer begin
	Layer3LteSetTimerForRrc(node, interfaceIndex, handoverParticipator.ueRnti,
		MSG_MAC_LTE_RELOCprep, RRC_LTE_DEFAULT_RELOC_PREP_TIME);
}


// /**
// FUNCTION   :: Layer3LteHandOverDecision
// LAYER      :: RRC
// PURPOSE    :: handover decision
// PARAMETERS ::
// + node             : Node*             : Pointer to node.
// + interfaceIndex   : int               : Interface index
// + ueRnti           : const LteRnti&    : the source of the report
// + measurementReport: std::list<MeasurementReport>*: report
// + targetRnti       : LteRnti*          : target eNB (if INVALID_RNTI
//                                          is set, doesn't hand over)
// RETURN     :: void : NULL
// **/
void Layer3LteHandOverDecision(
                    Node *node,
                    UInt32 interfaceIndex,
                    const LteRnti& ueRnti,
                    std::list<MeasurementReport>* measurementReport,
                    LteRnti* targetRnti)
{
    // station type guard
    LTE_LIB_STATION_TYPE_GUARD(node, interfaceIndex,
        LTE_STATION_TYPE_ENB, LTE_STRING_STATION_TYPE_ENB);

    LteRnti tmpTgtRnti = LTE_INVALID_RNTI;

    ERROR_Assert(measurementReport->size() > 0, "a measurement report msg"
        "should contains one or more reports");

    // at least, target eNB's RSRP needs to exceed serving eNB's RSRP.
    double maxRsrpValue = measurementReport->begin()
        ->measResults.measResultServCell.rsrpResult;
    for (std::list<MeasurementReport>::iterator it =					//Bang: This is array of measurement reports
        measurementReport->begin(); it != measurementReport->end(); ++it)
    {
        maxRsrpValue =
            max(it->measResults.measResultServCell.rsrpResult, maxRsrpValue);
    }

    // handover to eNB which has highest RSRP value
    for (std::list<MeasurementReport>::iterator it =
        measurementReport->begin(); it != measurementReport->end(); ++it)
    {
        ListMeasResultInfo& results = it->measResults.measResultNeighCells;
        for (ListMeasResultInfo::iterator it_result = results.begin();
            it_result != results.end(); ++it_result)
        {
            // skip nodes over our EPC subnet
            if (!EpcLteAppIsNodeOnTheSameEpcSubnet(
                node, it_result->rnti.nodeId))
            {
                continue;
            }

            // update max
            if (it_result->rsrpResult > maxRsrpValue)
            {
                maxRsrpValue = it_result->rsrpResult;
                tmpTgtRnti = it_result->rnti;
            }
        }
    }

    ERROR_Assert(tmpTgtRnti != LteLayer2GetRnti(node, interfaceIndex),
        "tried to handover to serving cell eNB");

    BOOL isDiscardHo = FALSE;
    LteConnectedInfo* connectedInfo =
        Layer3LteGetConnectedInfo(node, interfaceIndex, ueRnti);
    if (node->getNodeTime() <
        connectedInfo->connectedTime + RRC_LTE_DEFAULT_HO_IGNORED_TIME)
    {
        isDiscardHo = TRUE;
    }
    else
    {
        *targetRnti = tmpTgtRnti;
    }

#ifdef LTE_LIB_LOG
    char buf[MAX_STRING_LENGTH] = {0};
    if (isDiscardHo == TRUE)
    {
        sprintf(buf, "[handover decision is ignored]");
    }
    else
    {
        sprintf(buf, "[handover decision]");
    }

    {
        lte::LteLog::InfoFormat(node, interfaceIndex,
            LTE_STRING_LAYER_TYPE_RRC,
            LAYER3_LTE_MEAS_CAT_HANDOVER","
            LTE_STRING_FORMAT_RNTI","
            "%s,"
            LTE_STRING_FORMAT_HANDOVER_PARTICIPATOR,
            LTE_INVALID_RNTI.nodeId, LTE_INVALID_RNTI.interfaceIndex,
            buf,
            ueRnti.nodeId, ueRnti.interfaceIndex,
            node->nodeId, interfaceIndex,
            tmpTgtRnti.nodeId, tmpTgtRnti.interfaceIndex);
    }
#endif
}



// /**
// FUNCTION   :: Layer3LteReceiveHoReq
// LAYER      :: RRC
// PURPOSE    :: process after receive handover request
// PARAMETERS ::
// + node             : Node*             : Pointer to node.
// + interfaceIndex   : int               : Interface index
// + hoParticipator   : const HandoverParticipator&: participators of H.O.
// RETURN     :: void : NULL
// **/
void Layer3LteReceiveHoReq(
                    Node *node,
                    UInt32 interfaceIndex,
                    const HandoverParticipator& hoParticipator)
{
    // station type guard
    LTE_LIB_STATION_TYPE_GUARD(node, interfaceIndex,
        LTE_STATION_TYPE_ENB, LTE_STRING_STATION_TYPE_ENB);

#ifdef LTE_LIB_LOG
    {
        lte::LteLog::InfoFormat(node, interfaceIndex,
            LTE_STRING_LAYER_TYPE_RRC,
            LAYER3_LTE_MEAS_CAT_HANDOVER","
            LTE_STRING_FORMAT_RNTI","
            "[receive handover request],"
            LTE_STRING_FORMAT_HANDOVER_PARTICIPATOR,
            LTE_INVALID_RNTI.nodeId, LTE_INVALID_RNTI.interfaceIndex,
            hoParticipator.ueRnti.nodeId,
            hoParticipator.ueRnti.interfaceIndex,
            hoParticipator.srcEnbRnti.nodeId,
            hoParticipator.srcEnbRnti.interfaceIndex,
            hoParticipator.tgtEnbRnti.nodeId,
            hoParticipator.tgtEnbRnti.interfaceIndex);
    }
#endif

    // never receive H.O. req of the UE which is handingover from this eNB
    // to another eNB.
    LteConnectionInfo* connInfo = Layer3LteGetConnectionInfo(
        node, interfaceIndex, hoParticipator.ueRnti);
    BOOL isAcceptHoReq = connInfo == NULL ? TRUE : FALSE;
    BOOL admissionResult = Layer3LteAdmissionControl(
        node, interfaceIndex, hoParticipator);

    if (admissionResult == TRUE && isAcceptHoReq == TRUE)
    {
        // get connection config
        Layer2DataLte* layer2 =
            LteLayer2GetLayer2DataLte(node, interfaceIndex);
        RrcConnectionReconfiguration reconf;
        reconf.rrcConfig = *layer2->rrcConfig;

        // send HandoverRequestAck to source eNB
        EpcLteAppSend_HandoverRequestAck(
            node, interfaceIndex, hoParticipator, reconf);
        
        // prepare UE's connection info at handover info
        Layer3LteAddConnectionInfo(
            node, interfaceIndex, hoParticipator.ueRnti,
            LAYER3_LTE_CONNECTION_HANDOVER);
        Layer3LteSetHandoverParticipator(
            node, interfaceIndex, hoParticipator.ueRnti, hoParticipator);
        
        // set timer TX2WAIT_SN_STATUS_TRANSFER
        // (this should be after prepare UE's connction info)
        Layer3LteSetTimerForRrc(node, interfaceIndex, hoParticipator.ueRnti,
            MSG_MAC_LTE_X2_WaitSnStatusTransfer,
            RRC_LTE_DEFAULT_X2_WAIT_SN_STATUS_TRANSFER_TIME);
        // set timer TWAIT_ATTACH_UE_BY_HO
        Layer3LteSetTimerForRrc(node, interfaceIndex, hoParticipator.ueRnti,
            MSG_MAC_LTE_WaitAttachUeByHo,
            RRC_LTE_DEFAULT_WAIT_ATTCH_UE_BY_HO_TIME);

    }
    else
    {
        EpcLteAppSend_HoPreparationFailure(
            node, interfaceIndex, hoParticipator);
    }

    // update stats
    EpcData* epc = EpcLteGetEpcData(node);
    epc->statData.numHandoverRequestReceived++;
}



// /**
// FUNCTION   :: Layer3LtePrepareForHandoverExecution
// LAYER      :: RRC
// PURPOSE    :: clear information for handover execution
// PARAMETERS ::
// + node             : Node*             : Pointer to node.
// + interfaceIndex   : int               : Interface index
// + hoParticipator   : const HandoverParticipator&: participators of H.O.
// RETURN     :: void : NULL
// **/
void Layer3LtePrepareForHandoverExecution(Node* node, int interfaceIndex,
                    const HandoverParticipator& hoParticipator)
{
    // station type guard
    LTE_LIB_STATION_TYPE_GUARD(node, interfaceIndex,
        LTE_STATION_TYPE_UE, LTE_STRING_STATION_TYPE_UE);
    int phyIndex =
        LteGetPhyIndexFromMacInterfaceIndex(node, interfaceIndex);

#ifdef LTE_LIB_LOG
    {
        lte::LteLog::InfoFormat(node, interfaceIndex,
            LTE_STRING_LAYER_TYPE_RRC,
            LAYER3_LTE_MEAS_CAT_HANDOVER","
            LTE_STRING_FORMAT_RNTI","
            "[prepare for handover execution],"
            LTE_STRING_FORMAT_HANDOVER_PARTICIPATOR,
            LTE_INVALID_RNTI.nodeId, LTE_INVALID_RNTI.interfaceIndex,
            hoParticipator.ueRnti.nodeId,
            hoParticipator.ueRnti.interfaceIndex,
            hoParticipator.srcEnbRnti.nodeId,
            hoParticipator.srcEnbRnti.interfaceIndex,
            hoParticipator.tgtEnbRnti.nodeId,
            hoParticipator.tgtEnbRnti.interfaceIndex);
    }
#endif

    // these two routing processes should be done in the same simTime.
    // delete route to src eNB
    Layer3LteDeleteRoute(node, interfaceIndex, hoParticipator.srcEnbRnti);
    // add route
    Layer3LteAddRoute(node, interfaceIndex, hoParticipator.tgtEnbRnti);

    // clear each layer
    PhyLteClearInfo(node, phyIndex, hoParticipator.srcEnbRnti);
    PhyLteChangeState(node, phyIndex,
        (PhyDataLte*)node->phyData[phyIndex]->phyVar,
        PHY_LTE_IDLE_RANDOM_ACCESS);
    MacLteClearInfo(node, interfaceIndex, hoParticipator.srcEnbRnti);
    MacLteSetState(node, interfaceIndex,
        MAC_LTE_IDEL);

    // call reestablishment
    LteConnectedInfo* connInfo = Layer3LteGetConnectedInfo(
        node, interfaceIndex, hoParticipator.srcEnbRnti);
    for (MapLteRadioBearer::iterator it = connInfo->radioBearers.begin();
        it != connInfo->radioBearers.end(); ++it)
    {
        LteRlcReEstablishment(
            node, interfaceIndex, hoParticipator.srcEnbRnti, it->first,
            hoParticipator.tgtEnbRnti);
        PdcpLteReEstablishment(
            node, interfaceIndex, hoParticipator.srcEnbRnti, it->first);
    }

	//Bang: Print Handover
    // stop measurement
    PhyLteIFPHStopMeasIntraFreq(node, phyIndex);
    PhyLteIFPHStopMeasInterFreq(node, phyIndex);
    // reset
    Layer3LteMeasResetMeasurementValues(node, interfaceIndex);

    // don't clear Layer3Filtering. use these results after H.O.
    //// clear Layer3Filtering
    //LteLayer3Filtering* layer3Filtering =
    //    LteLayer2GetLayer3Filtering(node, interfaceIndex);
    //layer3Filtering->remove();

    // cancel all timer
    Layer3LteCancelAllTimerForRrc(node, interfaceIndex,
        hoParticipator.srcEnbRnti);

    // store connectedInfo (after this, eNB info is managed under target eNB)
    Layer3LteChangeConnectionState(
        node, interfaceIndex, hoParticipator.srcEnbRnti,
        LAYER3_LTE_CONNECTION_HANDOVER, hoParticipator);

    // change state
    Layer3LteSetState(node, interfaceIndex, LAYER3_LTE_RRC_IDLE);
}
