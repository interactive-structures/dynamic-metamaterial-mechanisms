#include "stdafx.h"

#include "CommunicatorNative.h"

NativeWrapper::CommunicatorNative::CommunicatorNative() : ManagedObject(new Communicator())
{
}

void NativeWrapper::CommunicatorNative::optimize()
{
	convertToNativeModel();

	pin_ptr<Eigen::Vector2d> results = m_Instance->optimizeNative();

	convertToNativeResult(results);
}

void NativeWrapper::CommunicatorNative::convertToNativeModel()
{
	for (size_t i = 0; i < modelNative->points->Length; i++)
	{
		m_Instance->addPoint(modelNative->points[i]->X, modelNative->points[i]->Y);
	}

	for (size_t i = 0; i < modelNative->cells->Length; i++)
	{
		GridCellNative^ native = modelNative->cells[i];
		m_Instance->addCell(native->a, native->b, native->c, native->d, native->type);
	}

	for (size_t i = 0; i < modelNative->anchors->Length; i++)
	{
		int anchor = modelNative->anchors[i];
		m_Instance->addAnchor(anchor);
	}

	for (size_t i = 0; i < modelNative->inputs->Length; i++)
	{
		int input = modelNative->inputs[i];
		m_Instance->addInput(input);

		for (size_t j = 0; j < modelNative->inputPaths[i]->Length; j++)
		{
			PointNative^ input = modelNative->inputPaths[i][j];
			m_Instance->addInputPathPoint(i, input->X, input->Y);
		}
	}

	for (size_t i = 0; i < modelNative->targets->Length; i++)
	{
		int target = modelNative->targets[i];
		m_Instance->addTarget(target);

		for (size_t j = 0; j < modelNative->targetPaths[i]->Length; j++)
		{
			PointNative^ target = modelNative->targetPaths[i][j];
			m_Instance->addTargetPathPoint(i, target->X, target->Y);
		}
	}
}

void NativeWrapper::CommunicatorNative::convertToNativeResult(Eigen::Vector2d* results)
{
	//this only works if all target paths have the same length
	int numResults = modelNative->targetPaths[0]->Length;
	int numPoints = modelNative->points->Length;
	int runner = 0;

	array<GridResultNative^> ^convertedResults = gcnew array<GridResultNative^>(numResults);

	for (size_t i = 0; i < numResults; i++)
	{
		array<PointNative^> ^convertedPoints = gcnew array<PointNative^>(numPoints);
		for (size_t j = 0; j < numPoints; j++)
		{
			auto& point = results[runner++];
			convertedPoints[j] = gcnew PointNative(point.x(), point.y());
		}
		convertedResults[i] = gcnew GridResultNative(convertedPoints);
	}


	resultNative = convertedResults;
}
