#pragma once

#include "ManagedObject.h"
#include "GridModelNative.h"
#include <Eigen/Dense>
#include "../../ShearCell_Optimization/src/GridModel.h"
#include "../../ShearCell_Optimization/src/Communicator.h"

using namespace System;

namespace NativeWrapper
{
	public ref class CommunicatorNative : public ManagedObject<Communicator>
	{
	public:
		CommunicatorNative();

		property GridModelNative ^modelNative;
		property array<GridResultNative^> ^resultNative;

		void optimize();

	private:
		void convertToNativeModel();
		void convertToNativeResult(Eigen::Vector2d* results);
	};
}