#pragma once

#include "ManagedObject.h"

using namespace System;

namespace NativeWrapper
{
	public ref class GridCellNative
	{
	public:
		//0 is rigid, 1 is shear, 2 is void, 3 is broken
		property int type;
		property int a;
		property int b;
		property int c;
		property int d;

		GridCellNative();
	};

	public ref class PointNative
	{
	public:
		PointNative(double x, double y);
		property double X;
		property double Y;
	};

	public ref class GridModelNative
	{

	public:
		GridModelNative();

		property array<PointNative^>^ points;
		property array<GridCellNative^>^ cells;

		property array<int>^ anchors;

		property array<int>^ inputs;
		property array<array<PointNative^>^>^ inputPaths;

		property array<int>^ targets;
		property array<array<PointNative^>^>^ targetPaths;
	};

	public ref class GridResultNative
	{
	public:
		GridResultNative(int numberResults);
		GridResultNative(array<PointNative^> ^pointParam);
		property array<PointNative^> ^points;
	};
};