using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.Simulation;
using ShearCell_Interaction.View;

namespace ShearCell_Interaction.Randomizer
{
    public abstract class Automaton
    {
        private int _iterationTimeStep = 200;
        private int _gridSize = 10;
        private int _numberIterations = 10;

        protected bool _isInitiallyRigid = false;
        protected List<Vector> _gridKeys;
        protected readonly MetamaterialModel _model;
        protected readonly ViewModel _viewModel;

        public Dictionary<Vector, CellType> Grid { get; private set; }
        public int CurrentIteration { get; private set; }
        public bool HasNextIteration { get; private set; }

        protected abstract int GetNeighborSum(Vector cellPosition);
        protected abstract CellType EvaluateRules(int neighborSum);

        public Automaton(MetamaterialModel model, ViewModel viewModel)
        {
            _model = model;
            _viewModel = viewModel;

            Grid = new Dictionary<Vector, CellType>(_gridSize * _gridSize);
            ConstructInitialGrid();

            CurrentIteration = 0;
            HasNextIteration = true;
        }

        public void SetInitialCells(List<Vector> initialCells)
        {
            var cellType = _isInitiallyRigid ? CellType.SHEAR : CellType.RIGID;

            foreach (var cellPosition in initialCells)
                Grid[cellPosition] = cellType;

            _gridKeys = Grid.Keys.ToList();

            Apply();
        }

        public void Run()
        {
            var worker = new BackgroundWorker();
            worker.DoWork += OnRandomizeInBackground;
            worker.RunWorkerCompleted += OnCompletedVisualizeRandomConfiguration;

            worker.RunWorkerAsync();
        }

        public void RunIteration()
        {
            if (!HasNextIteration)
                return;

            foreach (var cellPosition in _gridKeys)
            {
                var sum = GetNeighborSum(cellPosition);
                Grid[cellPosition] = EvaluateRules(sum);
            }

            CurrentIteration++;

            if (CurrentIteration >= _numberIterations - 1)
                HasNextIteration = false;
        }

        public void Apply()
        {
            foreach (var cellPosition in Grid.Keys)
            {
                var cellType = Grid[cellPosition];

                if (cellType == CellType.RIGID)
                    _model.AddCell(new RigidCell(), cellPosition, new Size(1, 1));
                else if (cellType == CellType.SHEAR)
                    _model.AddCell(new ShearCell(), cellPosition, new Size(1, 1));
            }

            _viewModel.Redraw();
        }

        protected static CellType GetRandomType(CellType[] candidates)
        {
            var random = new Random(DateTime.Now.Millisecond);
            var index = random.Next(candidates.Length);
            return candidates[index];
        }

        private void ConstructInitialGrid()
        {
            var initialType = _isInitiallyRigid ? CellType.RIGID : CellType.SHEAR;

            for (var x = 0; x < _gridSize; x++)
            for (var y = 0; y < _gridSize; y++)
                Grid.Add(new Vector(x, y), initialType);
        }

        private void OnRandomizeInBackground(object sender, DoWorkEventArgs e)
        {
            Debug.WriteLine("       GENERATE iteration: " + CurrentIteration);

            RunIteration();
            Thread.Sleep(_iterationTimeStep);
        }

        private void OnCompletedVisualizeRandomConfiguration(object sender, RunWorkerCompletedEventArgs e)
        {
            Debug.WriteLine("       SHOW iteration: " + CurrentIteration);

            Apply();

            var worker = (BackgroundWorker)sender;
            if (worker == null)
                return;

            if (HasNextIteration)
            {
                worker.RunWorkerAsync();
            }
            else
            {
                worker.DoWork -= OnRandomizeInBackground;
                worker.RunWorkerCompleted -= OnCompletedVisualizeRandomConfiguration;
                worker.Dispose();
            }
        }

    }
}
