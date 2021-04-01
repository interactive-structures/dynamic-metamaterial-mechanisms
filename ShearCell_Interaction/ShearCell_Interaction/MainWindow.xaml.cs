using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.Optimisation;
using ShearCell_Interaction.Presenter;
using ShearCell_Interaction.Randomizer;
using ShearCell_Interaction.Simulation;
using ShearCell_Interaction.Test;
using ShearCell_Interaction.View;

namespace ShearCell_Interaction
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow
    {
        private readonly MetamaterialPresenter _presenter;
        private readonly ViewModel _viewModel;

        private DateTime _lastMove;
        private Vector _lastMousePosition;
        private Vector _cellAreaStartPosition;
        private bool _isDraggingCellArea;
        private bool _isMouseDown;
        private bool _isShiftDown;
        private bool _isCtrlDown;
        private bool _isDraggingCanvas;
        private bool _justChangedPathSelection;

        private SimluationPathTests _simulationPathTest;
        private AutomaticPathSimulationTests _autoSimPathTest;

        public MainWindow()
        {
            var model = new MetamaterialModel();
            _viewModel = new ViewModel(model);
            _presenter = new MetamaterialPresenter(model, _viewModel);

            _simulationPathTest = new SimluationPathTests(_presenter, model, _viewModel, this);
            _autoSimPathTest = new AutomaticPathSimulationTests(_presenter, model, _viewModel, this);

            Height = ViewModel.WindowHeight + 120; //70 for controls bar + window frame
            Width = ViewModel.WindowWidth;

            DataContext = _viewModel;
            InitializeComponent();
            
            //OnRandomizeLargePatternsClicked(null, null);
        }

        private void OnWindowLoaded(object sender, RoutedEventArgs e)
        {
            var screenPosition = CoordinateConverter.ConvertGlobalToScreenCoordinates(new Vector(0, 0));

            var hintSize = 4;
            var offsetX = screenPosition.X + ViewModel.CellScale * 0.5 - hintSize * 0.5;
            var offsetY = screenPosition.Y - ViewModel.CellScale * 0.5 - hintSize * 0.5;

            var ellipse = new Ellipse()
            {
                Width = hintSize,
                Height = hintSize,
                Fill = Brushes.LightGray,
                Margin = new Thickness(offsetX, offsetY, 0, 0)
            };

            MainCanvas.Children.Add(ellipse);
        }

        private void ShowDebugPoint(Vector gridPosition, Brush fill)
        {
            if (fill == null)
                fill = Brushes.Red;

            var screenPosition = CoordinateConverter.ConvertGlobalToScreenCoordinates(gridPosition);

            var ellipse = new Ellipse()
            {
                Width = 10,
                Height = 10,
                Fill = fill,
                Stroke = Brushes.AntiqueWhite,
                StrokeThickness = 1,
                Margin = new Thickness(screenPosition.X - DebugPoint.Width * 0.5,
                                       screenPosition.Y - DebugPoint.Height * 0.5,
                                       0, 0)
            };

            MainCanvas.Children.Add(ellipse);
        }

        private void OnMouseUp(object sender, MouseButtonEventArgs e)
        {
            //MouseMove -= OnMouseMove;
            _isMouseDown = false;

            if (_isDraggingCanvas)
            {
                _isDraggingCanvas = false;
                return;
            }

            if (_viewModel.InteractionMode == InteractionMode.Simulate)
                return;

            var mousePosition = Mouse.GetPosition(MainCanvas);
            var mouseVector = new Vector(mousePosition.X, mousePosition.Y);

            if (mouseVector.Y < 0) //then it's outside of the canvas and doesn't need processing
                return;

            if (_viewModel.InteractionMode == InteractionMode.AddShearCell)
            {
                if (!_isShiftDown)
                    AddCellArea(mouseVector); //_presenter.CreateCell(mouseVector, typeof(ShearCell), new Size(1, 1));
                else
                    DeleteCellArea(mouseVector);

                _isDraggingCellArea = false;
                CellAreaPreviewContainer.Children.Clear();
            }
            else if (_viewModel.InteractionMode == InteractionMode.AddRigidCell)
            {
                if (!_isShiftDown)
                    AddCellArea(mouseVector); //_presenter.CreateCell(mouseVector, typeof(RigidCell), new Size(1, 1));
                else
                    DeleteCellArea(mouseVector);

                _isDraggingCellArea = false;
                CellAreaPreviewContainer.Children.Clear();
            }
            else if (_viewModel.InteractionMode == InteractionMode.AddAnchor ||
                     _viewModel.InteractionMode == InteractionMode.AddTracingPoint)
            {
                Point vertexCenterPosition;
                var success = TryGetVertexCenterPosition(e.OriginalSource, out vertexCenterPosition);

                if (!success)
                    return;

                var screenPosition = new Vector(vertexCenterPosition.X, vertexCenterPosition.Y);
                if (_viewModel.InteractionMode == InteractionMode.AddAnchor)
                    _presenter.AddAnchor(screenPosition);
                else if (_viewModel.InteractionMode == InteractionMode.AddTracingPoint)
                    _presenter.AddTracingPoint(screenPosition);
            }
            else if (_viewModel.InteractionMode == InteractionMode.SetTestInput)
            {
                if (_justChangedPathSelection)
                {
                    _justChangedPathSelection = false;
                    return;
                }


                if (DraggedSimulationPath.Visibility == Visibility.Visible)
                {
                    //snap to vertex
                    var gridCoordinates = CoordinateConverter.ConvertScreenToGlobalCellCoordinates(mouseVector);
                    gridCoordinates.X = Math.Round(gridCoordinates.X);
                    gridCoordinates.Y = Math.Round(gridCoordinates.Y);
                    var screenCoordinates = CoordinateConverter.ConvertGlobalToScreenCoordinates(gridCoordinates);
                    VisualizeDraggedSimulationPath(screenCoordinates);

                    var approximatedGeometry = DraggedSimulationPath.Data.GetFlattenedPathGeometry(0.001, ToleranceType.Absolute);
                    _presenter.SetInputPath(approximatedGeometry, DraggedSimulationPath.RenderTransform.Value);
                    DraggedSimulationPath.Visibility = Visibility.Hidden;
                    return;
                }

                Point vertexCenterPosition;
                var success = TryGetVertexCenterPosition(e.OriginalSource, out vertexCenterPosition);
                if (!success)
                    return;

                var screenPosition = new Vector(vertexCenterPosition.X, vertexCenterPosition.Y);
                _presenter.SetSimulationInputVertex(screenPosition);
            }
            else if (_viewModel.InteractionMode == InteractionMode.SetTestOutput)
            {
                if (_justChangedPathSelection)
                {
                    _justChangedPathSelection = false;
                    return;
                }

                if (DraggedSimulationPath.Visibility == Visibility.Visible)
                {
                    _presenter.SetOutputPath(
                        (PathGeometry)DraggedSimulationPath.Data.GetFlattenedPathGeometry(0.01,
                            ToleranceType.Absolute), DraggedSimulationPath.RenderTransform.Value);
                    DraggedSimulationPath.Visibility = Visibility.Hidden;
                    return;
                }

                Point vertexCenterPosition;
                var success = TryGetVertexCenterPosition(e.OriginalSource, out vertexCenterPosition);

                if (!success)
                    return;

                var screenPosition = new Vector(vertexCenterPosition.X, vertexCenterPosition.Y);
                _presenter.SetSimulationOutputVertex(screenPosition);
            }
        }

        private void OnMouseDown(object sender, MouseButtonEventArgs e)
        {
            _isMouseDown = true;

            var mousePosition = Mouse.GetPosition(MainCanvas);
            _lastMousePosition = new Vector(mousePosition.X, mousePosition.Y);

            //if (e.OriginalSource is Ellipse)
            //{
            //    Point vertexCenterPosition;
            //    var success = TryGetVertexCenterPosition(e.OriginalSource, out vertexCenterPosition);

            //    if (success && _viewModel.InteractionMode == InteractionMode.Simulate)
            //    {
            //        _presenter.SetSimulationInputVertex(new Vector(vertexCenterPosition.X, vertexCenterPosition.Y));
            //    }
            //}
            if (_isCtrlDown)
            {
                _isDraggingCanvas = true;
            }
            else if (_viewModel.InteractionMode == InteractionMode.AddShearCell || _viewModel.InteractionMode == InteractionMode.AddRigidCell)
            {
                _isDraggingCellArea = true;
                _cellAreaStartPosition = _lastMousePosition;
            }
        }

        private void OnMouseMove(object sender, MouseEventArgs e)
        {
            //Debug.WriteLine("move");
            var mousePosition = Mouse.GetPosition(MainCanvas);
            var mouseVector = new Vector(mousePosition.X, mousePosition.Y);

            var canvasOffset = MainCanvas.RenderTransform.Inverse as MatrixTransform;

            //Debug.WriteLine("mouse position X = {0}, Y = {1}", mousePosition.X, mousePosition.Y);
            //Debug.WriteLine("canvas offset  X = {0}, Y = {1}", canvasOffset.Matrix.OffsetX, canvasOffset.Matrix.OffsetY);

            //Debug.WriteLine("mouse.x < canvasoffset.x: {0}", mouseVector.X < canvasOffset.Matrix.OffsetX);

            //then it's outside of the canvas and doesn't need processing
            if (mouseVector.Y < canvasOffset.Matrix.OffsetY || mouseVector.X < canvasOffset.Matrix.OffsetX)
            {
                VisualHoverCell.Visibility = Visibility.Collapsed;
                return;
            }

            if (_isDraggingCanvas && e.OriginalSource is Canvas)
                DragCanvas(mouseVector);
            else if (_viewModel.InteractionMode == InteractionMode.AddShearCell || _viewModel.InteractionMode == InteractionMode.AddRigidCell)
                VisualizeCellHover(mouseVector);
            //else if (_viewModel.InteractionMode == InteractionMode.Simulate)
            //    HandleSimulation(mouseVector);
            else if (_viewModel.InteractionMode == InteractionMode.SetTestInput || _viewModel.InteractionMode == InteractionMode.SetTestOutput)
                VisualizeDraggedSimulationPath(mouseVector);

            _lastMousePosition = mouseVector;
        }

        private void DragCanvas(Vector currentMousePosition)
        {
            var difference = Vector.Subtract(currentMousePosition, _lastMousePosition);

            var currentTransform = MainCanvas.RenderTransform.Value;
            currentTransform.Append(new TranslateTransform(difference.X, difference.Y).Value);
            MainCanvas.RenderTransform = new MatrixTransform(currentTransform);

            //SimulationPaths.SetValue(Canvas.TopProperty, (double)SimulationPaths.GetValue(Canvas.TopProperty) - difference.Y);
            //SimulationPaths.SetValue(Canvas.RightProperty, (double)SimulationPaths.GetValue(Canvas.RightProperty) - difference.X);

            //Debug.WriteLine("Canvas transform: " + MainCanvas.RenderTransform.Value);
        }

        private void VisualizeCellHover(Vector mousePosition)
        {
            VisualHoverCell.Visibility = Visibility.Visible;

            var indexPosition = CoordinateConverter.ConvertScreenToCellIndex(mousePosition);
            Debug.WriteLine("hover:  mouse: " + mousePosition + "  index: " + indexPosition);

            var screenPosition = CoordinateConverter.ConvertGlobalToScreenCoordinates(indexPosition);

            var currentTransform = VisualHoverCell.RenderTransform.Value;
            var difference = Vector.Subtract(screenPosition, new Vector(currentTransform.OffsetX, currentTransform.OffsetY));

            currentTransform.Append(new TranslateTransform(difference.X, difference.Y - ViewModel.CellScale).Value);
            VisualHoverCell.RenderTransform = new MatrixTransform(currentTransform);

            //var visual = VisualHoverCell;
            //visual.Margin = new Thickness(
            //    screenPosition.X,
            //    screenPosition.Y - visual.Height,
            //    0, 0);

            if (_isDraggingCellArea)
                VisualizeDraggedCellArea(mousePosition);
        }

        private void AddCellArea(Vector mousePosition)
        {
            var startGridPosition = CoordinateConverter.ConvertScreenToCellIndex(_cellAreaStartPosition);
            var currentGridPosition = CoordinateConverter.ConvertScreenToCellIndex(mousePosition);

            Debug.WriteLine("   positon START = {0}, CURRENT = {1}", startGridPosition, currentGridPosition);

            var diffX = Math.Abs(currentGridPosition.X - startGridPosition.X);
            var diffY = Math.Abs(currentGridPosition.Y - startGridPosition.Y);

            Debug.WriteLine("   diff X = {0}, Y = {1}", diffX, diffY);

            var incrementFactorX = startGridPosition.X < currentGridPosition.X ? 1 : -1;
            var incrementFactorY = startGridPosition.Y < currentGridPosition.Y ? 1 : -1;

            Debug.WriteLine("   increment X = {0}, Y = {1}", incrementFactorX, incrementFactorY);

            CellAreaPreviewContainer.Children.Clear();

            for (var x = 0; x <= diffX; x++)
            {
                for (var y = 0; y <= diffY; y++)
                {
                    var currentCellPosition = new Vector(startGridPosition.X + x * incrementFactorX, startGridPosition.Y + y * incrementFactorY);
                    //var screenPosition = CoordinateConverter.ConvertGlobalToScreenCoordinates(currentCellPosition);
                    _viewModel.Model.DeleteCell(currentCellPosition);
                    //Debug.WriteLine("   current position: {0}", currentGridPosition);

                    if (_viewModel.InteractionMode == InteractionMode.AddShearCell)
                        _viewModel.Model.AddCell(new ShearCell(), currentCellPosition, new Size(1, 1), false);
                    else if (_viewModel.InteractionMode == InteractionMode.AddRigidCell)
                        _viewModel.Model.AddCell(new RigidCell(), currentCellPosition, new Size(1, 1), false);
                }
            }

            _viewModel.Model.UpdateConstraintsGraph();
            _viewModel.CurrentDoF = _viewModel.Model.CurrentDoF.ToString();
            _viewModel.Redraw();
        }

        private void DeleteCellArea(Vector mousePosition)
        {
            var startGridPosition = CoordinateConverter.ConvertScreenToCellIndex(_cellAreaStartPosition);
            var currentGridPosition = CoordinateConverter.ConvertScreenToCellIndex(mousePosition);

            var diffX = Math.Abs(currentGridPosition.X - startGridPosition.X);
            var diffY = Math.Abs(currentGridPosition.Y - startGridPosition.Y);


            var incrementFactorX = startGridPosition.X < currentGridPosition.X ? 1 : -1;
            var incrementFactorY = startGridPosition.Y < currentGridPosition.Y ? 1 : -1;

            CellAreaPreviewContainer.Children.Clear();

            for (var x = 0; x <= diffX; x++)
            {
                for (var y = 0; y <= diffY; y++)
                {
                    var currentCellPosition = new Vector(startGridPosition.X + x * incrementFactorX, startGridPosition.Y + y * incrementFactorY);
                    //var screenPosition = CoordinateConverter.ConvertGlobalToScreenCoordinates(currentCellPosition);
                    _viewModel.Model.DeleteCell(currentCellPosition);
                }
            }

            _viewModel.Model.UpdateConstraintsGraph();
            _viewModel.CurrentDoF = _viewModel.Model.CurrentDoF.ToString();
            _viewModel.Redraw();
        }

        //TODO refactor!

        private void VisualizeDraggedCellArea(Vector mousePosition)
        {
            var startGridPosition = CoordinateConverter.ConvertScreenToCellIndex(_cellAreaStartPosition);
            var currentGridPosition = CoordinateConverter.ConvertScreenToCellIndex(mousePosition);

            Debug.WriteLine("positon START = {0}, CURRENT = {1}", startGridPosition, currentGridPosition);

            var diffX = Math.Abs(currentGridPosition.X - startGridPosition.X);
            var diffY = Math.Abs(currentGridPosition.Y - startGridPosition.Y);

            Debug.WriteLine("diff X = {0}, Y = {1}", diffX, diffY);

            var incrementFactorX = startGridPosition.X < currentGridPosition.X ? 1 : -1;
            var incrementFactorY = startGridPosition.Y < currentGridPosition.Y ? 1 : -1;

            Debug.WriteLine("increment X = {0}, Y = {1}", incrementFactorX, incrementFactorY);

            CellAreaPreviewContainer.Children.Clear();

            for (var x = 0; x <= diffX; x++)
            {
                for (var y = 0; y <= diffY; y++)
                {
                    var currentCellPosition = new Vector(startGridPosition.X + x * incrementFactorX, startGridPosition.Y + y * incrementFactorY);
                    //Debug.WriteLine("current position: {0}", currentGridPosition);
                    var screenPosition = CoordinateConverter.ConvertGlobalToScreenCoordinates(currentCellPosition);

                    var cellPreview = new Rectangle()
                    {
                        Stroke = Brushes.LightGray,
                        StrokeThickness = 1,
                        Width = ViewModel.CellScale,
                        Height = ViewModel.CellScale,
                        Margin = new Thickness(screenPosition.X, screenPosition.Y - ViewModel.CellScale, 0, 0)
                    };

                    CellAreaPreviewContainer.Children.Add(cellPreview);
                }
            }
        }

        private bool TryGetVertexCenterPosition(object originalSource, out Point vertexCenterPosition)
        {
            if (!(originalSource is Polygon))
            {
                vertexCenterPosition = new Point();
                return false;
            }

            var vertexVisual = (Polygon)originalSource;
            var points = new List<Point>(vertexVisual.Points);
            var minY = points.Min(p => p.Y);
            var minX = points.Min(p => p.X);
            var maxX = points.Max(p => p.X);

            var halfSize = maxX > minX ? Math.Abs(maxX - minX) * 0.5 : Math.Abs(minX - maxX) * 0.5;

            vertexCenterPosition = new Point(minX + halfSize, minY + halfSize);

            Debug.WriteLine("vertex position: " + vertexCenterPosition);

            return true;
        }

        private void OnExportClicked(object sender, RoutedEventArgs e)
        {
            _presenter.ExportCurrentConfiguration();
        }

        private void OnImportClicked(object sender, RoutedEventArgs e)
        {
            _presenter.ImportConfiguration();
        }

        private void OnSaveImageClicked(object sender, RoutedEventArgs e)
        {
            _presenter.SaveCurrentCanvas(this);
        }

        //private void OnPathSimulationClicked(object sender, RoutedEventArgs e)
        //{
        //    _presenter.StartPathSimulation();
        //}

        private void OnSimulationPathSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            _justChangedPathSelection = true;

            //_viewModel.InteractionMode = InteractionMode.SetTestInput;

            var allPaths = (ListBox)sender;
            var item = (ListBoxItem)allPaths.SelectedItems[0];
            var selectedInputPath = (Path)item.Content;

            DraggedSimulationPath.Data = selectedInputPath.Data;
            DraggedSimulationPath.Visibility = Visibility.Visible;
        }

        private void VisualizeDraggedSimulationPath(Vector mouseVector)
        {
            //setting transform rather than thickness so all data can be read from the Path object for creating the input path 

            var currentTransform = DraggedSimulationPath.RenderTransform.Value;
            var currentTranslate = new Vector(currentTransform.OffsetX, currentTransform.OffsetY);
            var translateOffset = Vector.Subtract(mouseVector, currentTranslate);

            currentTransform.Append(new TranslateTransform(translateOffset.X, translateOffset.Y).Value);
            DraggedSimulationPath.RenderTransform = new MatrixTransform(currentTransform);
        }

        private void OnKeyUp(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.LeftShift || e.Key == Key.RightShift)
            {
                _isShiftDown = false;
                Debug.WriteLine("shift down: " + _isShiftDown);
                return;
            }

            if (e.Key == Key.LeftCtrl || e.Key == Key.RightCtrl)
            {
                _isCtrlDown = false;
                return;
            }

            if (e.Key == Key.OemPlus || e.Key == Key.Add || e.Key == Key.OemMinus || e.Key == Key.Subtract)
            {
                if (_isCtrlDown)
                    ScaleCanvas(e.Key);
                else
                    ScaleSimulationPath(e.Key);

                return;
            }

            if (e.Key == Key.T)
                _presenter.TraceAllPoints();
            else if (e.Key == Key.Z)
                _presenter.ResetDeformation(false);
            else if (e.Key == Key.R)
                _presenter.Reset();
            else if (e.Key == Key.E)
                ExportDoorLatch();
            else if (e.Key == Key.C)
                _presenter.Clear();
            //else if (e.Key == Key.D)
            //    Debug.WriteLine("Object has {0} DoF ({1} connected components)", _viewModel.Model.GetConstraintsGraph().Count - 1, _viewModel.Model.GetConstraintsGraph().Count);
            else if (e.Key == Key.P)
            {
                if (!_simulationPathTest.Started)
                    _simulationPathTest.Start();
                else
                    _simulationPathTest.NextStep();
            }
            else if (e.Key == Key.I)
            {
                _autoSimPathTest.Start();
            }
            else if (e.Key == Key.O)
                _simulationPathTest.RunSimulationAndStoreSteps();
            else if (e.Key == Key.Space)
                AddRandomCell();
            else if (e.Key == Key.Left)
                OnSimulationStepBack(null, null);
            else if (e.Key == Key.Right)
                OnSimulationStepForward(null, null);
            else if (e.Key == Key.Space)
                OnSimulationPlayPause(null, null);
            else if (e.Key == Key.J)
                _presenter.SaveCurrentCanvas(this);
            else if (e.Key == Key.K)
                _presenter.Scale(2);
            else if(e.Key == Key.D1)
            {            
                var existingVertex = _viewModel.Model.Vertices.Find(current => MathHelper.IsEqualDouble(current.X, 0) && MathHelper.IsEqualDouble(current.Y, 0));

                if (existingVertex == null)
                    return;

                _viewModel.Model.InputVertex = existingVertex;
                _viewModel.HighlightInputVertex();

                var path = CurveDrawingHelper.GetCubicPolynomial(100, .2, new Vector(0, 0));
                //var path = CurveDrawingHelper.GetFoliumDescartes(100, .7, new Vector(0, 0));
                //var path = CurveDrawingHelper.GetTransformedBicorn(100, 1.0, new Vector(0, 0));
                //var path = CurveDrawingHelper.GetBowCurve(100, 1.0, new Vector(0, 0));
                //var path = CurveDrawingHelper.GetLine(100, 2.0, new Vector(0, 0));
                _presenter.SetInputPath(path, Matrix.Identity);
            }
            else if (e.Key == Key.D2)
            {
                AutoOptimization autoOpt = new AutoOptimization(_viewModel,_viewModel.Model, _presenter);
                autoOpt.Start();
            }

        }

        private void AddRandomCell()
        {
            _presenter.AddRandomCell();
        }

        private void OnKeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.LeftShift || e.Key == Key.RightShift)
                _isShiftDown = true;
            else if (e.Key == Key.LeftCtrl || e.Key == Key.RightCtrl)
                _isCtrlDown = true;

            //Debug.WriteLine("shift down: " + _isShiftDown);
        }

        private void ScaleSimulationPath(Key key)
        {
            if (_viewModel.InteractionMode != InteractionMode.SetTestInput && _viewModel.InteractionMode != InteractionMode.SetTestOutput)
                return;

            const double scale = 0.1;
            var scaleFactor = 1.0;

            if (key == Key.OemPlus || key == Key.Add)
                scaleFactor += scale;
            else if (key == Key.OemMinus || key == Key.Subtract)
                scaleFactor -= scale;

            var currentTransform = DraggedSimulationPath.RenderTransform.Value;
            currentTransform.Append(new ScaleTransform(scaleFactor, scaleFactor).Value);

            DraggedSimulationPath.RenderTransform = new MatrixTransform(currentTransform);
        }

        private void ScaleCanvas(Key key)
        {
            const double scale = 0.2;
            var scaleFactor = 1.0;

            if (key == Key.OemPlus || key == Key.Add)
                scaleFactor += scale;
            else if (key == Key.OemMinus || key == Key.Subtract)
                scaleFactor -= scale;

            var currentTransform = MainCanvas.RenderTransform.Value;
            currentTransform.Append(new ScaleTransform(scaleFactor, scaleFactor).Value);

            MainCanvas.RenderTransform = new MatrixTransform(currentTransform);
        }

        private void OnRandomizeClicked(object sender, RoutedEventArgs e)
        {
            //_presenter.RandomizeCellConfiguration();
            _presenter.RandomizeAllCellConfigurations();
        }

        private void OnRandomizeLargePatternsClicked(object sender, RoutedEventArgs e)
        {
            //_viewModel.Model.Clear();

            //for (var dimension = 5; dimension <= 5; dimension++)
            //{
            //    var randomizer = new LargePatternRandomizer(dimension, dimension, _viewModel, _viewModel.Model);
            //    randomizer.MinDoF = 4;
            //    randomizer.MaxDoF = 6;
            //    randomizer.MinRigidCells = (int)(dimension * dimension * 0.05);
            //    randomizer.MaxRigidCells = (int)(dimension * dimension * 0.3);
            //    randomizer.GeneratePersistedVariations();
            //}

            var randomizer = new LargePatternRandomizer(1, 1, _viewModel, _viewModel.Model);
            //randomizer.SystematicallyRandomizePatterns(6, 6 + 3 * 5, 3);
            randomizer.SystematicallyRandomizePatterns(5, 6, 1, true);
        }

        private void ExportDoorLatch()
        {
            ExportTestCase.ExportDoorLatch(45, -1, _viewModel.Model, _presenter, _viewModel);
        }

        private void OnOptimizedPathSimulationClicked(object sender, RoutedEventArgs e)
        {
            _presenter.StartOptimizedPathSimulation();
        }

        private void OnSimulationPlayPause(object sender, RoutedEventArgs e)
        {
            _presenter.ToggleSimulationPlaying();
        }

        private void OnSimulationStepBack(object sender, RoutedEventArgs e)
        {
            _presenter.StepSimulationStepsBack(1);
        }

        private void OnSimulationStepForward(object sender, RoutedEventArgs e)
        {
            _presenter.StepSimulationStepsForward(1);
        }

        private void OnShowConnectedComponentsClicked(object sender, RoutedEventArgs e)
        {
            _presenter.ToggleConnectedComponents();
        }

        private void OnTraceAllClicked(object sender, RoutedEventArgs e)
        {
            _presenter.TraceAllPoints();
        }

        private void OnResetDeformationClicked(object sender, RoutedEventArgs e)
        {
            _presenter.ResetDeformation();
        }

        private void OnResetConfigClicked(object sender, RoutedEventArgs e)
        {
            _presenter.Reset();
        }

        private void OnEraseClicked(object sender, RoutedEventArgs e)
        {
            _presenter.Clear();
        }

        private void OnMarkNonShearing(object sender, RoutedEventArgs e)
        {
            _presenter.ToggleMarkNonShearingCells();
        }
        private void OnSetNonShearing(object sender, RoutedEventArgs e)
        {
            _viewModel.Model.SetNonShearingCells();
            _viewModel.Redraw();
        }

        private void OnScaleClicked(object sender, RoutedEventArgs e)
        {
            _presenter.Scale(2);
        }
        private void OnDecreaseScaleClicked(object sender, RoutedEventArgs e)
        {
            _presenter.DecreaseScale();
        }

        private void OnOptimizePatternClicked(object sender, RoutedEventArgs e)
        {
            _presenter.StartOptimization();
        }

        private void OnOptimizePatternHierarchicalClicked(object sender, RoutedEventArgs e)
        {
            _presenter.StartHierarchicalOptimization();
        }

        private void OnGraphMergeClicked(object sender, RoutedEventArgs e)
        {
            if (_viewModel.Model.ConstraintGraph.Count < 2)
                return;

            int cc1, cc2;
            var success = TryPickRandomComponents(out cc1, out cc2);
            if (!success)
                return;

            Debug.WriteLine("# connected components: {0}, index 1 = {1}, index 2 = {2}", _viewModel.Model.ConstraintGraph.Count, cc1, cc2);

            var mergeCandidates = _viewModel.Model.GetCandidatesForMerging(_viewModel.Model.ConstraintGraph[cc1], _viewModel.Model.ConstraintGraph[cc2]);
            Debug.WriteLine("# merge candidates: {0}", mergeCandidates.Count);

            if (mergeCandidates.Count < 1)
                return;

            var random = new Random();
            var candidateIndex = random.Next(0, mergeCandidates.Count);
            var cell = mergeCandidates[candidateIndex];
            Debug.WriteLine(">>> merge at {0}", cell);
            _viewModel.Model.AddCell(new RigidCell(), cell.IndexVertex.ToInitialVector(), new Size(1, 1));


            //_viewModel.Model.TryMergeComponents(_viewModel.Model.ConstraintGraph[cc1], _viewModel.Model.ConstraintGraph[cc2]);
            //Debug.WriteLine("after merge: CC = {0}", _viewModel.Model.ConstraintGraph.Count);

            _viewModel.VisualizeConnectedComponents();
            _viewModel.MarkNonShearingCells();
            _viewModel.Redraw();
        }

        private void OnGraphSplitClicked(object sender, RoutedEventArgs e)
        {
            var random = new Random(DateTime.Now.Millisecond);
            //var componentIndex = random.Next(0, _viewModel.Model.ConstraintGraph.Count);
            //Debug.WriteLine("# connected components: {0}, index 1 = {1}", _viewModel.Model.ConstraintGraph.Count, componentIndex);

            double rigidityThreshold = 0.1;
            HashSet<HashSet<Edge>> component = null;

            var i = 0;
            foreach (var comp in _viewModel.Model.ConstraintGraph)
            {
                var rigidity = _viewModel.Model.GetRigidityOfComponent(comp);
                Debug.WriteLine("component [{0}] rigidity = {1:0.00}", i, rigidity);

                if (rigidity >= rigidityThreshold)
                {
                    component = comp;
                    break;
                }

                i++;
            }

            if (component == null)
            {
                Debug.WriteLine("No component with rigidity >= {0} found. Return.", rigidityThreshold);
                return;
            }

            var splitCandidates = _viewModel.Model.GetCandidatesForSplitting(component);
            Debug.WriteLine("# split candidates: {0}", splitCandidates.Count);

            if (splitCandidates.Count < 1)
                return;

            var candidateIndex = random.Next(0, splitCandidates.Count);

            var splitModeIndex = random.Next(0, 2);
            var splitMode = (SplitMode)splitModeIndex;

            Debug.WriteLine("Split component with {0} cells at {1} with mode {2}", component.Count, splitCandidates[candidateIndex], splitMode);
            _viewModel.Model.SplitComponentAt(component, splitCandidates[candidateIndex], splitMode);

            //var initialDoF = _viewModel.Model.CurrentDoF;

            //while (initialDoF >= _viewModel.Model.CurrentDoF)
            //{
            //    var candidateIndex = random.Next(0, splitCandidates.Count);
            //    var cell = splitCandidates[candidateIndex];
            //    Debug.WriteLine(">>> split at {0}", cell);

            //    _viewModel.Model.AddCell(new ShearCell(), cell.IndexVertex.ToInitialVector(), new Size(1, 1));
            //    splitCandidates.Remove(cell);
            //}

            _viewModel.Redraw();
        }

        private bool TryPickRandomComponents(out int componentIndex1, out int componentIndex2)
        {
            componentIndex1 = -1;
            componentIndex2 = -1;

            if (_viewModel.Model.ConstraintGraph.Count < 2)
                return false;

            var random = new Random(DateTime.Now.Millisecond);
            componentIndex1 = random.Next(0, _viewModel.Model.ConstraintGraph.Count);

            while (componentIndex2 < 0 || componentIndex2 == componentIndex1)
                componentIndex2 = random.Next(0, _viewModel.Model.ConstraintGraph.Count);

            return true;
        }

        private void OnTempTestClicked(object sender, RoutedEventArgs e)
        {
            //TestFindNonShearingAreas();
            //TestFindClosestArea();

            //for (var i = 0; i < _viewModel.Model.ConstraintGraph.Count; i++)
            //{
            //    var component = _viewModel.Model.ConstraintGraph[i];

            //    var size = _viewModel.Model.GetSizeOfComponent(component);
            //    var rigidity = _viewModel.Model.GetRigidityOfComponent(component);

            //    Debug.WriteLine("component: # cells = {0}, rigidity = {1}", size, rigidity);
            //}

            //var bitcode = _viewModel.Model.GetEncoding();

            var code = "110011110011110011110011111011111011";
            BitcodeHelper.CodeToModel(code, _viewModel.Model);
            _viewModel.Redraw();

            ////visual debugging only
            //List<Cell> cells;
            //List<Vector> positions;
            //var width = _viewModel.Model.Vertices.Max(v => v.InitialX);

            //var randomizer = new LargePatternRandomizer(width, _viewModel.Model.Cells.Count / width, _viewModel, _viewModel.Model);
            //randomizer.CreateCellConfiguration(bitcode, new Vector(width + 2, 0), out cells, out positions);

            //for (var i = 0; i < cells.Count; i++)
            //    _viewModel.Model.AddCell(cells[i], positions[i], new Size(1, 1), false);

            //_viewModel.Model.UpdateConstraintsGraph();
            //_viewModel.Redraw();
        }

        private void TestFindNonShearingAreas()
        {
            for (var i = 0; i < _viewModel.Model.ConstraintGraph.Count; i++)
                _viewModel.Model.GetNonShearingAreasInComponent(_viewModel.Model.ConstraintGraph[i]);
        }

        private void TestFindClosestArea()
        {
            var anchors = _viewModel.Model.GetAnchors();
            if (anchors.Count <= 0)
            {
                Debug.WriteLine(" ---> no anchors!");
                return;
            }

            for (var i = 0; i < _viewModel.Model.ConstraintGraph.Count; i++)
            {
                Vertex closestVertex;
                Cell closestCell;
                HashSet<Cell> closestNonShearingArea;

                var success = _viewModel.Model.GetClosestNonShearingAreaToAnchor(_viewModel.Model.ConstraintGraph[i], anchors[0], out closestNonShearingArea, out closestCell, out closestVertex);
                if (!success)
                {
                    Debug.WriteLine(" ---> no success!");
                    continue;
                }

                Debug.WriteLine("for component with {0} cells, found ", _viewModel.Model.ConstraintGraph[i].Count);
                Debug.WriteLine("   closest Cell: " + closestCell);
                Debug.WriteLine("   closest Vertex: " + closestVertex);
                Debug.WriteLine("   closest Area with {0} cells.", closestNonShearingArea.Count);
            }
        }
        private void OnDrawingButtonClicked(object sender, RoutedEventArgs e)
        {
            DrawingCanvas.Visibility = Visibility.Visible;
        }
        
        private void OnDrawingButtonUnchecked(object sender, RoutedEventArgs e)
        {
            if (DrawingCanvas.Visibility == Visibility.Visible
                && _viewModel.InteractionMode != InteractionMode.DrawInput
                && _viewModel.InteractionMode != InteractionMode.DrawOutput)
            {
                DrawingCanvas.Visibility = Visibility.Hidden;
            }
        }

        private void DrawingCanvasStrokeCollected(object sender, InkCanvasStrokeCollectedEventArgs e)
        {
            PolylineHelper.ResampleStrokeSimple(e.Stroke);

            var renderTransformInverse = MainCanvas.RenderTransform.Inverse;

            if (renderTransformInverse == null)
                return;
            
            var points = e.Stroke.GetBezierStylusPoints();
            var sampledPoints = PolylineHelper.SamplePolygon(points.Select(p => renderTransformInverse.Transform(p.ToPoint())).ToList(),MetamaterialPresenter.GetPathSamplingFactor());
            
            if (_viewModel.InteractionMode == InteractionMode.DrawInput)
            {
                _viewModel.Model.InputPath = sampledPoints;
                _viewModel.DrawInputPath();
            }
            else if (_viewModel.InteractionMode == InteractionMode.DrawOutput)
            {
                _viewModel.Model.OutputPath = sampledPoints;
                _viewModel.DrawOutputPath();
            }
            DrawingCanvas.Visibility = Visibility.Hidden;
            DrawingCanvas.Strokes.Clear();
        }
    }
}
