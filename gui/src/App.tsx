import { useState } from 'react'
import './App.css'

type ViewMode = 'single' | 'dual' | 'quad'

const Screen = ({ id, title }: { id: number; title: string }) => (
  <div className="bg-gray-800 rounded-lg p-8 shadow-lg flex flex-col items-center justify-center h-full min-h-96">
    <h2 className="text-2xl font-bold mb-4">{title}</h2>
    <p className="text-gray-300 text-center">Content for screen {id}</p>
  </div>
)

function App() {
  const [viewMode, setViewMode] = useState<ViewMode>('quad')
  const [selectedScreens, setSelectedScreens] = useState<number[]>([1, 2])
  const [singleScreen, setSingleScreen] = useState<number>(1)
  const [isControlsExpanded, setIsControlsExpanded] = useState<boolean>(true)

  const availableScreens = [
    { id: 1, title: 'Camera Feed' },
    { id: 2, title: 'Sonar' },
    { id: 3, title: 'Control Panel' },
    { id: 4, title: 'Telemetry' },
  ]

  const toggleScreenSelection = (screenId: number) => {
    setSelectedScreens((prev) => {
      if (prev.includes(screenId)) {
        return prev.filter((id) => id !== screenId)
      } else if (prev.length < 2) {
        return [...prev, screenId]
      }
      return prev
    })
  }

  const renderViewMode = () => {
    if (viewMode === 'single') {
      const screen = availableScreens.find((s) => s.id === singleScreen)!
      return (
        <div className="grid grid-cols-1 gap-8 flex-1">
          <Screen id={screen.id} title={screen.title} />
        </div>
      )
    }

    if (viewMode === 'dual') {
      return (
        <div className="grid grid-cols-2 gap-8 flex-1">
          {selectedScreens.map((screenId) => {
            const screen = availableScreens.find((s) => s.id === screenId)!
            return <Screen key={screenId} id={screen.id} title={screen.title} />
          })}
        </div>
      )
    }

    // quad mode
    return (
      <div className="grid grid-cols-2 gap-8 flex-1">
        {availableScreens.map((screen) => (
          <Screen key={screen.id} id={screen.id} title={screen.title} />
        ))}
      </div>
    )
  }

  return (
    <div className="min-h-screen bg-gray-900 text-white flex flex-col p-8">
      <div className="flex items-center justify-center gap-6 mb-8">
        <h1 className="text-6xl font-bold">
          NEU Marine Robotics
        </h1>
        <button
          onClick={() => setIsControlsExpanded(!isControlsExpanded)}
          className="bg-gray-800 hover:bg-gray-700 transition-all rounded-lg px-4 py-2 font-semibold text-lg h-fit"
          title="Toggle Controls"
        >
          <span className={`inline-block transition-transform duration-300 ${isControlsExpanded ? 'rotate-180' : ''}`}>
            ▼
          </span>
        </button>
      </div>

      {/* Collapsible Controls */}
      {isControlsExpanded && (
        <div className="bg-gray-800 rounded-lg shadow-lg mb-8 p-6">
          {/* View Mode Selection */}
          <div className="mb-6">
            <h3 className="text-lg font-semibold mb-3">View Mode</h3>
            <div className="flex gap-4">
              <button
                onClick={() => setViewMode('single')}
                className={`px-4 py-2 rounded-lg font-medium transition-all ${
                  viewMode === 'single'
                    ? 'bg-blue-600 text-white'
                    : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                }`}
              >
                Single View
              </button>
              <button
                onClick={() => setViewMode('dual')}
                className={`px-4 py-2 rounded-lg font-medium transition-all ${
                  viewMode === 'dual'
                    ? 'bg-blue-600 text-white'
                    : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                }`}
              >
                Dual View
              </button>
              <button
                onClick={() => setViewMode('quad')}
                className={`px-4 py-2 rounded-lg font-medium transition-all ${
                  viewMode === 'quad'
                    ? 'bg-blue-600 text-white'
                    : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                }`}
              >
                2x2 View
              </button>
            </div>
          </div>

          {/* Screen Selection for Single View */}
          {viewMode === 'single' && (
            <div>
              <h3 className="text-lg font-semibold mb-3">Select Screen</h3>
              <div className="flex gap-2 flex-wrap">
                {availableScreens.map((screen) => (
                  <button
                    key={screen.id}
                    onClick={() => setSingleScreen(screen.id)}
                    className={`px-4 py-2 rounded-lg font-medium transition-all ${
                      singleScreen === screen.id
                        ? 'bg-blue-600 text-white'
                        : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                    }`}
                  >
                    {screen.title}
                  </button>
                ))}
              </div>
            </div>
          )}

          {/* Screen Selection for Dual View */}
          {viewMode === 'dual' && (
            <div>
              <h3 className="text-lg font-semibold mb-3">
                Select Screens ({selectedScreens.length}/2)
              </h3>
              <div className="flex gap-2 flex-wrap">
                {availableScreens.map((screen) => (
                  <button
                    key={screen.id}
                    onClick={() => toggleScreenSelection(screen.id)}
                    className={`px-4 py-2 rounded-lg font-medium transition-all ${
                      selectedScreens.includes(screen.id)
                        ? 'bg-blue-600 text-white'
                        : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                    }`}
                  >
                    {screen.title}
                  </button>
                ))}
              </div>
              {selectedScreens.length < 2 && (
                <p className="text-yellow-400 text-sm mt-2">
                  Select 2 screens to display
                </p>
              )}
            </div>
          )}
        </div>
      )}

      {/* Grid Display */}
      {viewMode === 'dual' && selectedScreens.length < 2 ? (
        <div className="flex items-center justify-center flex-1 text-gray-400">
          <p className="text-xl">Please select 2 screens to display</p>
        </div>
      ) : (
        renderViewMode()
      )}
    </div>
  )
}

export default App
