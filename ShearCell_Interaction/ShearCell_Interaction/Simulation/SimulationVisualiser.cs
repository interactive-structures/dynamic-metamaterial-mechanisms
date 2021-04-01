using System.Collections.Generic;
using System.ComponentModel;
using System.Threading;
using System.Windows;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.View;

namespace ShearCell_Interaction.Simulation
{
    public class SimulationVisualiser
    {
        private readonly ViewModel _viewModel;
        private List<List<Vector>> _framePositions;
        private int _currentFrame;
        private int _lastReadPoint;

        public bool IsPaused { get; private set; }
        public bool LoopAnimation { get; set; }

        public bool SimulationAvailable
        {
            get { return _framePositions != null && _framePositions.Count > 0; }
        }

        private BackgroundWorker _worker;

        public SimulationVisualiser( ViewModel viewModel)
        {
            _viewModel = viewModel;

            LoopAnimation = true;
        }

        public void OptimizationInfoAvailable(List<List<Vector>> framePositions)
        {
            _framePositions = framePositions;

            _currentFrame = 0;
            IsPaused = true;

            if (_worker != null)
            {
                Clear();
            }

            _worker = new BackgroundWorker();
            _worker.DoWork += OnSimulateFrame;
            _worker.RunWorkerCompleted += OnCompletedSimulatingFrame;

            PlayPauseSimulation();
        }
        public void Clear()
        {
            if (_worker == null)
                return;

            _worker.DoWork -= OnSimulateFrame;
            _worker.RunWorkerCompleted -= OnCompletedSimulatingFrame;
            _worker = null;
        }

        public void ResetSimulationStep()
        {
            _currentFrame = 0;
            //IsPaused = true;
        }

        public void PlayPauseSimulation()
        {
            if (_worker == null)
                return;

            IsPaused = !IsPaused;

            if (_worker.IsBusy)
                return;

            if (!IsPaused)
                _worker.RunWorkerAsync();
        }

        public void StepSimulation(int numberFrames)
        {
            _currentFrame = Helper.MathHelper.Mod(_currentFrame + numberFrames, _framePositions.Count);
            PlayPauseSimulation();
            IsPaused = true;
        }

        private void OnSimulateFrame(object sender, DoWorkEventArgs e)
        {
            for (var i = 0; i < _framePositions[_currentFrame].Count; i++)
                _viewModel.Model.Vertices[i].SetPosition((_framePositions[_currentFrame])[i]);

            Thread.Sleep(100);
        }

        private void OnCompletedSimulatingFrame(object sender, RunWorkerCompletedEventArgs e)
        {
            if (_currentFrame == 0)
                Application.Current.Dispatcher.Invoke(_viewModel.ResetTracingPaths);

            Application.Current.Dispatcher.Invoke(_viewModel.RedrawDeformed);

            if (IsPaused)
                return;

            _currentFrame++;

            if (_currentFrame >= _framePositions.Count)
            {
                _currentFrame = 0;

                if (!LoopAnimation)
                {
                    IsPaused = true;
                    return;
                }
            }

            _worker.RunWorkerAsync();
        }
    }
}