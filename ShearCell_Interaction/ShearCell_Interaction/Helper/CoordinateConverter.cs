using System;
using System.Diagnostics;
using System.Windows;
using ShearCell_Interaction.View;

namespace ShearCell_Interaction.Helper
{
    public class CoordinateConverter
    {
        private static int OffsetX = (int)(ViewModel.WindowWidth * 0.25);
        private static int OffsetY = (int)(-ViewModel.WindowHeight * 0.25);

        public static Vector ConvertScreenToCellIndex(Vector screenPosition)
        {
            var gridPosition = ConvertScreenToGlobalCellCoordinates(screenPosition);
            return new Vector(Math.Floor(gridPosition.X), Math.Floor(gridPosition.Y));
        }

        public static Vector ConvertScreenToGlobalCellCoordinates(Vector screenPosition)
        {
            return new Vector((screenPosition.X - OffsetX) / ViewModel.CellScale,
                             (ViewModel.WindowHeight - screenPosition.Y + OffsetY) / ViewModel.CellScale);
        }

        public static Point ConvertScreenToGlobalCellCoordinates(Point screenPosition)
        {
            return new Point((screenPosition.X - OffsetX) / ViewModel.CellScale,
                             (ViewModel.WindowHeight - screenPosition.Y + OffsetY) / ViewModel.CellScale);
        }

        public static Vector ConvertGlobalToScreenCoordinates(Vector globalGridPosition)
        {
            return new Vector(globalGridPosition.X * ViewModel.CellScale + OffsetX,
                             (ViewModel.WindowHeight - globalGridPosition.Y * ViewModel.CellScale + OffsetY));
        }

        
        //public static Point ConvertScreenToCellIndex(Point screenPosition)
        //{
        //    var gridPosition = ConvertScreenToGlobalCellCoordinates(screenPosition);
        //    return new Point(Math.Floor(gridPosition.X), Math.Floor(gridPosition.Y));
        //}

        //public static Point ConvertScreenToGlobalCellCoordinates(Point screenPosition)
        //{
        //    return new Point((screenPosition.X - OffsetX) / ViewModel.CellScale,
        //                     (ViewModel.WindowHeight - screenPosition.Y - OffsetY) / ViewModel.CellScale);
        //    //return new Point((screenPosition.X - OffsetX) / ViewModel.CellScale,
        //    //                 (ViewModel.WindowHeight - screenPosition.Y - OffsetY) / ViewModel.CellScale);
        //}

        //public static Point ConvertGlobalToScreenCoordinates(Point globalGridPosition)
        //{
        //    return new Point(globalGridPosition.X * ViewModel.CellScale + OffsetX,
        //                     (ViewModel.WindowHeight - globalGridPosition.Y * ViewModel.CellScale + OffsetY));
        //    //return new Point(globalGridPosition.X * ViewModel.CellScale + OffsetX,
        //    //                 ViewModel.WindowHeight - (globalGridPosition.Y * ViewModel.CellScale + OffsetY));
        //}

    }
}
