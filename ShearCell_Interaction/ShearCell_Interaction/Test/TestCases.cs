using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Media;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.Presenter;
using ShearCell_Interaction.Simulation;
using ShearCell_Interaction.View;

namespace ShearCell_Interaction.Test
{
    public class TestCases
    {

        public static void TestSingleCellSquareMoveDiagonal(MetamaterialPresenter presenter, MetamaterialModel model, MainWindow window)
        {
            var cellSize = new Size(1, 1);
            //var offset = new Point(0.2, 0.2);
            var offset = new Vector(0.5, 0.5);
            var anchorIndex = 0;
            var movedIndex = 2;

            //TestSingleShearCell(movedIndex, offset, new List<int> { anchorIndex }, cellSize, presenter, model, window);
            var inputCell = AddTestCell(typeof(ShearCell), cellSize, new List<int> { anchorIndex }, presenter, model);
            AddTestDeformation(inputCell, movedIndex, offset, presenter, window);
        }

        public static void TestSingleCellSquareMoveAdjacent(MetamaterialPresenter presenter, MetamaterialModel model, MainWindow window)
        {
            var cellSize = new Size(1, 1);
            //var offset = new Point(0.2, 0.2);
            var offset = new Vector(1, 0.5);
            var anchorIndex = 3;
            var movedIndex = 2;

            //TestSingleShearCell(movedIndex, offset, new List<int> { anchorIndex }, cellSize, presenter, model, window);
            var inputCell = AddTestCell(typeof(ShearCell), cellSize, new List<int> { anchorIndex }, presenter, model);
            AddTestDeformation(inputCell, movedIndex, offset, presenter, window);
        }

        public static void TestSingleCellSquareDoubleAnchored(MetamaterialPresenter presenter, MetamaterialModel model, MainWindow window)
        {
            var cellSize = new Size(1, 1);
            //var offset = new Point(0.2, 0.2);
            var offset = new Vector(0.1, 0.1);
            //var anchorIndex = 3;
            var movedIndex = 0;

            //TestSingleShearCell(movedIndex, offset, new List<int> { 2, 3 }, cellSize, presenter, model, window);
            var inputCell = AddTestCell(typeof(ShearCell), cellSize, new List<int> { 2, 3 }, presenter, model);
            AddTestDeformation(inputCell, movedIndex, offset, presenter, window);
        }

        public static void TestSingleCellRectangularMoveDiagonal(MetamaterialPresenter presenter, MetamaterialModel model, MainWindow window)
        {
            var cellSize = new Size(2, 1);
            var offset = new Vector(0.4, 0.2);
            //var offset = new Point(0.8, 0.4);
            var anchorIndices = new List<int> {1};
            var movedIndex = 3;

            //TestSingleShearCell(movedIndex, offset, new List<int>{anchorIndex}, cellSize, presenter, model, window);

            var inputCell = AddTestCell(typeof(ShearCell), cellSize, anchorIndices, presenter, model);
            AddTestDeformation(inputCell, movedIndex, offset, presenter, window);
        }

        public static void TestSingleRigidCell(MetamaterialPresenter presenter, MetamaterialModel model, MainWindow window)
        {
            var cellSize = new Size(2, 1);
            var offset = new Vector(0.4, 0.2);
            //var offset = new Point(0.8, 0.4);
            var anchorIndices = new List<int> { 3 };
            var movedIndex = 0;

            var inputCell = AddTestCell(typeof(RigidCell), cellSize, anchorIndices, presenter, model);
            AddTestDeformation(inputCell, movedIndex, offset, presenter, window);
        }

        public static void TestShearRigidCell(MetamaterialPresenter presenter, MetamaterialModel model, MainWindow window)
        {
            var cellSize = new Size(1, 1);
            var position = new Vector(ViewModel.CellScale * 10, ViewModel.WindowHeight - ViewModel.CellScale * 10);

            var offset = new Vector(0.2, 0.2);
            //var offset = new Point(0.8, 0.4);
            var anchorIndices = new List<int> { 0, 1 };
            var movedIndex = 2;

            var inputCell = AddTestCell(typeof(ShearCell), cellSize, position, anchorIndices, presenter, model);
            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X - ViewModel.CellScale, position.Y), new List<int>(), presenter, model);

            AddTestDeformation(inputCell, movedIndex, offset, presenter, window);
        }

        public static void Test4CheckerboardCells(MetamaterialPresenter presenter, MetamaterialModel model, MainWindow window)
        {
            var cellSize = new Size(1, 1);
            var position = new Vector(ViewModel.CellScale * 2, ViewModel.WindowHeight - ViewModel.CellScale * 2);

            var offset = new Vector(0.0, -0.4);
            //var offset = new Point(0.8, 0.4);
            var anchorIndices = new List<int> { 0 };
            var movedIndex = 2;

            AddTestCell(typeof(ShearCell), cellSize, position, anchorIndices, presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X + ViewModel.CellScale, position.Y - ViewModel.CellScale), new List<int>(), presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X + ViewModel.CellScale * 2, position.Y - ViewModel.CellScale * 2), new List<int>(), presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X, position.Y - ViewModel.CellScale * 2), new List<int>(), presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X + ViewModel.CellScale * 2, position.Y), new List<int>(), presenter, model);

            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X + ViewModel.CellScale, position.Y), new List<int>(), presenter, model);
            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X, position.Y - ViewModel.CellScale), new List<int>(), presenter, model);
            var inputCell = AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X + ViewModel.CellScale, position.Y - ViewModel.CellScale * 2), new List<int>(), presenter, model);
            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X + ViewModel.CellScale * 2, position.Y - ViewModel.CellScale), new List<int>(), presenter, model);

            AddTestDeformation(inputCell, movedIndex, offset, presenter, window);
        }

        public static void TestLinearCompress(MetamaterialPresenter presenter, MetamaterialModel model, MainWindow window)
        {
            var cellSize = new Size(1, 1);
            var position = new Vector(ViewModel.CellScale * 10, ViewModel.WindowHeight - ViewModel.CellScale * 10);

            var offset = new Vector(0.0, -0.4);
            //var offset = new Point(0.8, 0.4);
            var anchorIndices = new List<int> { 0, 1 };
            var movedIndex = 2;

            AddTestCell(typeof(RigidCell), cellSize, position, new List<int>(), presenter, model);
            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X + ViewModel.CellScale, position.Y), anchorIndices, presenter, model);

            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X, position.Y - ViewModel.CellScale), new List<int>(), presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X, position.Y - ViewModel.CellScale * 2), new List<int>(), presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X + ViewModel.CellScale, position.Y - ViewModel.CellScale), new List<int>(), presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X + ViewModel.CellScale, position.Y - ViewModel.CellScale * 2), new List<int>(), presenter, model);

            var inputCell = AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X, position.Y - ViewModel.CellScale * 3), new List<int>(), presenter, model);
            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X + ViewModel.CellScale, position.Y - ViewModel.CellScale * 3), new List<int>(), presenter, model);

            AddTestDeformation(inputCell, movedIndex, offset, presenter, window);
        }

        public static void Test4CheckerboardCellsInputPathSpiralForward(MetamaterialPresenter presenter, MetamaterialModel model, ViewModel viewModel)
        {
            var canvasHeight = ViewModel.WindowHeight - 80;
            var cellSize = new Size(1, 1);
            //var position = new Vector(ViewModel.CellScale * 2, canvasHeight - ViewModel.CellScale * 4);
            var position = new Vector(ViewModel.CellScale * 2, ViewModel.WindowHeight - ViewModel.CellScale * 4);
            //ViewModel.CellScale = 75.0;

            var anchorIndices = new List<int> { 0 };
            var movedIndex = 2;
            //var outputIndex = 3;

            var inputCell = AddTestCell(typeof(ShearCell), cellSize, position, anchorIndices, presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X + ViewModel.CellScale, position.Y - ViewModel.CellScale), new List<int>(), presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X + ViewModel.CellScale * 2, position.Y - ViewModel.CellScale * 2), new List<int>(), presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X, position.Y - ViewModel.CellScale * 2), new List<int>(), presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X + ViewModel.CellScale * 2, position.Y), new List<int>(), presenter, model);

            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X + ViewModel.CellScale, position.Y), new List<int>(), presenter, model);
            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X, position.Y - ViewModel.CellScale), new List<int>(), presenter, model);
            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X + ViewModel.CellScale, position.Y - ViewModel.CellScale * 2), new List<int>(), presenter, model);
            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X + ViewModel.CellScale * 2, position.Y - ViewModel.CellScale), new List<int>(), presenter, model);


            var translateOffset = new Vector(-0.5, 0.5);
            var gridTranslate = Vector.Add(translateOffset, inputCell.CellVertices[movedIndex].ToVector());
            var screenTranslate = CoordinateConverter.ConvertGlobalToScreenCoordinates(gridTranslate);
            var inputVertexScreenPosition = CoordinateConverter.ConvertGlobalToScreenCoordinates(inputCell.CellVertices[movedIndex].ToInitialVector());

            var pathGeometry = Application.Current.FindResource("PathSpiral") as PathGeometry;
            var transform = new Matrix();
            transform.Scale(1.4, 1.4);
            transform.Translate(screenTranslate.X, screenTranslate.Y);
            
            viewModel.InteractionMode = InteractionMode.Simulate;

            presenter.SetSimulationInputVertex(inputVertexScreenPosition);
            presenter.SetInputPath(pathGeometry, transform);
            presenter.TraceAllPoints();
            //presenter.StartPathSimulation();
        }

        public static void Test4CheckerboardCellsInputPathSpiralInverse(MetamaterialPresenter presenter, MetamaterialModel model, ViewModel viewModel)
        {
            var canvasHeight = ViewModel.WindowHeight - 80;
            var cellSize = new Size(1, 1);
            //var position = new Vector(ViewModel.CellScale * 2, canvasHeight - ViewModel.CellScale * 4);
            var position = new Vector(ViewModel.CellScale * 2, ViewModel.WindowHeight - ViewModel.CellScale * 4);
            //ViewModel.CellScale = 75.0;

            var anchorIndices = new List<int> {0};
            var movedIndex = 3;
            //var outputIndex = 3;

            AddTestCell(typeof(ShearCell), cellSize, position, anchorIndices, presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X + ViewModel.CellScale, position.Y - ViewModel.CellScale), new List<int>(), presenter, model);
            var inputCell = AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X + ViewModel.CellScale * 2, position.Y - ViewModel.CellScale * 2), new List<int>(), presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X, position.Y - ViewModel.CellScale * 2), new List<int>(), presenter, model);
            AddTestCell(typeof(ShearCell), cellSize, new Vector(position.X + ViewModel.CellScale * 2, position.Y), new List<int>(), presenter, model);

            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X + ViewModel.CellScale, position.Y), new List<int>(), presenter, model);
            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X, position.Y - ViewModel.CellScale), new List<int>(), presenter, model);
            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X + ViewModel.CellScale, position.Y - ViewModel.CellScale * 2), new List<int>(), presenter, model);
            AddTestCell(typeof(RigidCell), cellSize, new Vector(position.X + ViewModel.CellScale * 2, position.Y - ViewModel.CellScale), new List<int>(), presenter, model);


            var translateOffset = new Vector(-0.5, 0.5);
            var gridTranslate = Vector.Add(translateOffset, inputCell.CellVertices[movedIndex].ToVector());
            var screenTranslate = CoordinateConverter.ConvertGlobalToScreenCoordinates(gridTranslate);
            var inputVertexScreenPosition = CoordinateConverter.ConvertGlobalToScreenCoordinates(inputCell.CellVertices[movedIndex].ToInitialVector());

            var pathGeometry = Application.Current.FindResource("PathSpiral") as PathGeometry;
            var transform = new Matrix();
            transform.Scale(1.5, 1.5);
            transform.Translate(screenTranslate.X, screenTranslate.Y);

            viewModel.InteractionMode = InteractionMode.Simulate;

            presenter.SetSimulationInputVertex(inputVertexScreenPosition);
            presenter.SetInputPath(pathGeometry, transform);
            presenter.TraceAllPoints();
            //presenter.StartPathSimulation();
        }

        private static Cell AddTestCell(Type cellType, Size cellSize, List<int> anchoredVertexIndices, MetamaterialPresenter presenter, MetamaterialModel model)
        {
            return AddTestCell(cellType, cellSize, new Vector(ViewModel.WindowWidth*0.5, ViewModel.WindowHeight*0.5), anchoredVertexIndices, presenter, model);
        }

        private static Cell AddTestCell(Type cellType, Size cellSize, Vector screenPosition, List<int> anchoredVertexIndices, MetamaterialPresenter presenter, MetamaterialModel model)
        {
            //presenter.CreateCell(new Point(ViewModel.WindowWidth * 0.5, ViewModel.WindowHeight * 0.5), cellType, cellSize);            
            presenter.CreateCell(screenPosition, cellType, cellSize);
            var cell = model.Cells[model.Cells.Count - 1];
            //var indexVertex = inputCell.IndexVertex;

            foreach (var anchorIndex in anchoredVertexIndices)
                presenter.AddAnchor(CoordinateConverter.ConvertGlobalToScreenCoordinates(cell.CellVertices[anchorIndex].ToVector()));

            return cell;
        }

        private static void AddTestDeformation(Cell inputCell, int movedVertexIndex, Vector offset, MetamaterialPresenter presenter, MainWindow window)
        {
            var targetVertex = inputCell.CellVertices[movedVertexIndex];

            //TODO remove relative offset!
            var targetPoint = CoordinateConverter.ConvertGlobalToScreenCoordinates(Vector.Add(targetVertex.ToVector(), offset));

            window.DebugPoint.Visibility = Visibility.Visible;
            window.DebugPoint.Margin = new Thickness(targetPoint.X - window.DebugPoint.Width * 0.5,
                                                     targetPoint.Y - window.DebugPoint.Height * 0.5,
                                                     0, 0);
            window.DeformedView.Visibility = Visibility.Visible;

            presenter.SetSimulationInputVertex(CoordinateConverter.ConvertGlobalToScreenCoordinates(targetVertex.ToVector()));
            //presenter.UpdateDeformation(targetPoint);
        }
    }
}
