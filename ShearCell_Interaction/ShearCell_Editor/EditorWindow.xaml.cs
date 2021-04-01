using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using ShearCell_Data.Helper;
using ShearCell_Data.Model;
using ShearCell_Data.View;
using ShearCell_Editor.Event;
using ShearCell_Editor.Presenter;

namespace ShearCell_Editor
{
    public partial class EditorWindow
    {
        private readonly EditorViewModel _viewModel;

        private bool _isMouseDown;
        private EditorPresenter _presenter;
        private PathDrawingPresenter _drawingPresenter;

        //private MetamaterialModel _model;
        //private ViewModel _viewModel;
        //private MetamaterialPresenter _presenter;

        public EditorWindow()
        {
            ////var model = new MetamaterialModel();
            //_model = new MetamaterialModel();
            //_viewModel = new ViewModel(_model);
            //_presenter = new MetamaterialPresenter(_model, _viewModel);

            //Height = ViewModel.WindowHeight + 120; //70 for controls bar + window frame
            //Width = ViewModel.WindowWidth;

            //DataContext = _viewModel;
            //InitializeComponent();

            InitializeComponent();

            var model = new MetamaterialModel();
            _viewModel = new EditorViewModel(GridViewport.Viewport);
            _presenter = new EditorPresenter(model, _viewModel);
            _drawingPresenter = new PathDrawingPresenter(_viewModel, DrawingCanvas);
            _drawingPresenter.OnPathCollected += OnPathCollected;

            DataContext = _viewModel;
            Loaded += OnWindowLoaded;
        }

        void OnWindowLoaded(object sender, RoutedEventArgs e)
        {
            //GridViewport.ZoomExtents(500);
            GridViewport.IsRotationEnabled = EditorViewModel.Is3DEnabled;
            GridViewport.Focus();
        }

        //protected override void OnClosed(EventArgs e)
        //{
        //    //this._viewModel.Save("MyModel.xml");
        //    base.OnClosed(e);
        //}

        protected override void OnKeyDown(KeyEventArgs e)
        {
            base.OnKeyDown(e);
            switch (e.Key)
            {
                case Key.A:
                    GridViewport.ZoomExtents(500);
                    break;
                case Key.C:
                    _viewModel.Clear();
                    break;
                case Key.L:
                    GridViewport.IsRotationEnabled = !GridViewport.IsRotationEnabled;
                    break;
            }
        }

        private void view1_MouseDown(object sender, MouseButtonEventArgs e)
        {
            _isMouseDown = true;

            if (_viewModel.InteractionMode == InteractionMode.MarkBoundary)
                _viewModel.Viewport.CaptureMouse();

            if (_viewModel.InteractionMode == InteractionMode.Draw)
                _drawingPresenter.HandleMouseDown(e.GetPosition(GridViewport));


            //var screenPosition = e.GetPosition(GridViewport);
            //_viewModel.AddCellOnGrid(screenPosition);
            //GridViewport.CaptureMouse();



            //bool shift = Keyboard.IsKeyDown(Key.LeftShift);
            //var screenPosition = e.GetPosition(GridViewport);

            //Vector3D n;
            //var source = FindSource(screenPosition, out n);
            //if (source != null)
            //{
            //    if (shift)
            //        _viewModel.Remove(source);
            //    else
            //        _viewModel.Add(source, n);
            //}
            //else
            //{
            //    var ray = GridViewport.Viewport.Point2DtoRay3D(screenPosition);
            //    if (ray != null)
            //    {
            //        var planeIntersection = ray.PlaneIntersection(new Point3D(0, 0, 0.5), new Vector3D(0, 0, 1));
            //        if (planeIntersection.HasValue)
            //        {
            //            var pRound = new Point3D(Math.Round(planeIntersection.Value.X), Math.Round(planeIntersection.Value.Y), 0);
            //            _viewModel.AddVoxel(pRound);
            //        }
            //    }
            //}
            //_viewModel.UpdateCellPreview(Mouse.GetPosition(GridViewport));
            ////CaptureMouse();
        }

        private void view1_MouseMove(object sender, MouseEventArgs e)
        {

            if (_isMouseDown)
            {
                if (_viewModel.InteractionMode == InteractionMode.MarkBoundary)
                    _presenter.UpdateBoundary(e.GetPosition(GridViewport));
            }
            else
            {
                if (_viewModel.InteractionMode == InteractionMode.MarkBoundary)
                    _viewModel.UpdateCellPreview(Mouse.GetPosition(GridViewport));
                else if (_viewModel.InteractionMode == InteractionMode.AddAnchor)
                    _viewModel.UpdateAnchorPreview(Mouse.GetPosition(GridViewport));

            }

            if (_viewModel.InteractionMode == InteractionMode.Draw)
                _drawingPresenter.HandleMouseMove(e.GetPosition(GridViewport));
        }

        private void view1_MouseUp(object sender, MouseButtonEventArgs e)
        {
            _isMouseDown = false;
            _viewModel.Viewport.ReleaseMouseCapture();

            if (_viewModel.InteractionMode == InteractionMode.Draw)
                _drawingPresenter.HandleMouseUp(e.GetPosition(GridViewport));

            //ReleaseMouseCapture();
        }

        private void view1_MouseLeave(object sender, MouseEventArgs e)
        {
            _viewModel.RemoveCellPreview();
            _viewModel.RemoveAnchorPreview();
        }

        private void view1_KeyUp(object sender, KeyEventArgs e)
        {
            // Should update preview voxel when shift is released
            _viewModel.UpdateCellPreview(Mouse.GetPosition(GridViewport));
        }

        private void view1_KeyDown(object sender, KeyEventArgs e)
        {
            // Should update preview voxel when shift is pressed
            _viewModel.UpdateCellPreview(Mouse.GetPosition(GridViewport));
        }


        private void OnPreviewMotionClicked(object sender, RoutedEventArgs e)
        {
            throw new NotImplementedException();
        }

        private void OnGenerateCellsClicked(object sender, RoutedEventArgs e)
        {
            throw new NotImplementedException();
        }

        private void OnMarkBoundaryClicked(object sender, RoutedEventArgs e)
        {
            //    //TODO integrate into ViewModel!

            //    var p = Mouse.GetPosition(GridViewport);
            //    var hits = Viewport3DHelper.FindHits(GridViewport.Viewport, p);

            //    //var source = FindSource(p, out n);
        }

        //void UpdatePreview()
        //{
        //    var screenPosition = Mouse.GetPosition(GridViewport);
        //    bool shift = (Keyboard.IsKeyDown(Key.LeftShift));
        //    Vector3D normal;

        //    var source = FindSource(screenPosition, out normal);
        //    if (shift)
        //    {
        //        _viewModel.PreviewVoxel(null);
        //        _viewModel.HighlightVoxel(source);
        //    }
        //    else
        //    {
        //        _viewModel.PreviewVoxel(source, normal);
        //        _viewModel.HighlightVoxel(null);
        //    }
        //}

        private void OnPathCollected(object sender, PathCollectedEventArgs e)
        {
            LinesVisual3D line = new LinesVisual3D();
            line.Thickness = 2.0;
            line.Color = DrawingCanvas.DefaultDrawingAttributes.Color;

            line.Points = new Point3DCollection();

            for (var index = 0; index < e.PathPoints.Count; index++)
            {
                var point3D = TryGetPoint3D(e.PathPoints[index]);

                if (point3D.HasValue)
                    line.Points.Add(point3D.Value);

                if (e.PathPoints.Count > 2 && index < e.PathPoints.Count - 1)
                {
                    var nexPoint3D = TryGetPoint3D(e.PathPoints[index + 1]);

                    if (nexPoint3D.HasValue)
                        line.Points.Add(nexPoint3D.Value);
                }
            }

            GridViewport.Children.Add(line);

            DrawingCanvas.Strokes.Remove(e.Stroke);
        }

        public Point3D? TryGetPoint3D(Point point)
        {
            var ray = GridViewport.Viewport.Point2DtoRay3D(point);
            if (ray == null)
                return null;

            var planeIntersection = ray.PlaneIntersection(new Point3D(0, 0, .5), new Vector3D(0, 0, 0.1));
            if (!planeIntersection.HasValue)
                return null;

            return planeIntersection.Value;
        }

        Model3D FindSource(Point screenPosition, out Vector3D normal)
        {
            //var hits = Viewport3DHelper.FindHits(GridViewport.Viewport, screenPosition);
            var hits = GridViewport.Viewport.FindHits(screenPosition);

            foreach (var hit in hits)
            {
                if (hit.Model == _viewModel.PreviewModel)
                    continue;
                normal = hit.Normal;
                return hit.Model;
            }

            normal = new Vector3D();
            return null;
        }

    }
}
