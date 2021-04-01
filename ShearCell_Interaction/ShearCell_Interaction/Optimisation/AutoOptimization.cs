using System;
using System.Collections.Generic;
using System.Windows;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.Presenter;
using ShearCell_Interaction.Simulation;
using ShearCell_Interaction.View;

namespace ShearCell_Interaction.Optimisation
{
    public class AutoOptimization
    {
        private readonly ViewModel _viewModel;
        private readonly MetamaterialModel _model;
        private readonly MetamaterialPresenter _presenter;

        private const int NUM_CURVES = 4;
        private const int GRID_SIDE_LENGTH = 12;

        private CsvLogger _logger;

        public AutoOptimization(ViewModel viewModel, MetamaterialModel model, MetamaterialPresenter presenter)
        {
            _viewModel = viewModel;
            _model = model;
            _presenter = presenter;

            _logger = new CsvLogger("log.csv");
        }

        public void Start()
        {
            _viewModel.NumScalesOptimization = 2;

            _viewModel.MaxNumberPathSamples = 100;
            _viewModel.MinNumberPathSamples = 100;
            _viewModel.NumPathDivides = 0;
            _viewModel.NumberIterationsGeneration = 1000;

            for (int i = 0; i < NUM_CURVES; i++)
            {
                ResetModelForOptimizer(GRID_SIDE_LENGTH, GRID_SIDE_LENGTH);

                _model.InputPath = GetCurveForState(i);

                var optimizer = new HierarchicalOptimization(_model, _viewModel, _presenter);
                _logger.AddOrChangePrefix(i.ToString("D"), this);
                optimizer.Logger = _logger;

                optimizer.Start();

                _presenter.ExportCurrentConfiguration("auto_" + i + "_solution");

                Console.WriteLine("Done for curve " + i);
            }
        }

        private void ResetModelForOptimizer(int sizeX, int sizeY)
        {
            _model.Clear();

            for (int x = 0; x < sizeX; x++)
            {
                for (int y = 0; y < sizeY; y++)
                {
                    _model.AddCell(new ShearCell(), new Vector(x, y), new Size(1, 1), false);
                }
            }

            _model.UpdateConstraintsGraph();

            var inputVertex = _model.Vertices.Find(current =>
                MathHelper.IsEqualDouble(current.X, 0) && MathHelper.IsEqualDouble(current.Y, 0));

            if (inputVertex == null)
                throw new Exception("No cell at 0/0.");

            _model.InputVertex = inputVertex;

            var outputVertex = _model.Vertices.Find(current =>
                MathHelper.IsEqualDouble(current.X, sizeX - 1) && MathHelper.IsEqualDouble(current.Y, sizeY - 1));

            if (outputVertex == null)
                throw new Exception("No cell at 0/0.");

            _model.OutputVertex = outputVertex;
            _model.OutputPath = CurveDrawingHelper.GetLine(100, 2.0, new Vector(sizeX - 1, sizeY - 1));

            _model.SetAnchor(new Vector(4, 4), true);
            _model.SetAnchor(new Vector(4, 5), true);
            _model.SetAnchor(new Vector(5, 5), true);
            _model.SetAnchor(new Vector(5, 4), true);
        }

        public List<Vector> GetCurveForState(int state)
        {
            switch (state)
            {
                case 0:
                    return CurveDrawingHelper.GetCubicPolynomial(100, .2, new Vector(0, 0));
                case 1:
                    return CurveDrawingHelper.GetFoliumDescartes(100, .7, new Vector(0, 0));
                case 2:
                    return CurveDrawingHelper.GetTransformedBicorn(100, 1.0, new Vector(0, 0));
                case 3:
                    return CurveDrawingHelper.GetBowCurve(100, 1.0, new Vector(0, 0));
            }

            throw new Exception("No path for curve ID " + state);
        }
    }
}