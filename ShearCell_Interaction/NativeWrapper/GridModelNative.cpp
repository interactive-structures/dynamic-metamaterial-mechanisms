#include "stdafx.h"

#include "GridModelNative.h"

NativeWrapper::GridModelNative::GridModelNative()
{
}

NativeWrapper::GridCellNative::GridCellNative()
{
}

NativeWrapper::GridResultNative::GridResultNative(int numberResults)
{
	points = gcnew array<PointNative^>(numberResults);
}

NativeWrapper::GridResultNative::GridResultNative(array<PointNative^> ^pointsParam)
{
	points = pointsParam;
}


NativeWrapper::PointNative::PointNative(double x, double y)
{
	X = x;
	Y = y;
}
