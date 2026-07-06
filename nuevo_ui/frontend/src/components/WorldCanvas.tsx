/**
 * WorldCanvas — live 2D world-frame map.
 *
 * Card shell and toggle legend styled to match OdometryCard (SensorSection).
 * Canvas style (translucent dark, subtle grid) is the original WorldCanvas design.
 */
import { useRef, useEffect, useState, useCallback } from 'react'
import { useRobotStore, LIDAR_WINDOW_FRAMES } from '../store/robotStore'

// Venue: 5×6 grid, 610 mm cells.
// Origin (0,0) sits half a grid below the venue bottom border.
// → venue spans x: -305..2745, y: 305..3965
const GRID_MM        = 610
const VENUE_COLS     = 5
const VENUE_ROWS     = 6
const VENUE_LEFT_MM  = -GRID_MM / 2                          // -305
const VENUE_RIGHT_MM =  VENUE_LEFT_MM + VENUE_COLS * GRID_MM // 2745
const VENUE_BOTTOM_MM = GRID_MM / 2                          // 305
const VENUE_TOP_MM   =  VENUE_BOTTOM_MM + VENUE_ROWS * GRID_MM // 3965
const VENUE_PAD_MM   =  GRID_MM / 2                          // 305

const VENUE_EXT = {
  minX: VENUE_LEFT_MM  - VENUE_PAD_MM,
  maxX: VENUE_RIGHT_MM + VENUE_PAD_MM,
  minY: VENUE_BOTTOM_MM - GRID_MM,
  maxY: VENUE_TOP_MM   + VENUE_PAD_MM,
}
const MAP_WIDTH_MM = VENUE_EXT.maxX - VENUE_EXT.minX
const MAP_HEIGHT_MM = VENUE_EXT.maxY - VENUE_EXT.minY

interface Trails { odom: boolean; gps: boolean; fused: boolean; lidar: boolean; virtual: boolean }

const SERIES = [
  { key: 'odom'  as const, label: 'Odometry', color: '#60a5fa' },
  { key: 'fused' as const, label: 'Fused',    color: '#4ade80' },
  { key: 'gps'   as const, label: 'GPS',      color: '#facc15' },
  { key: 'lidar' as const, label: 'Lidar',    color: '#f87171' },
  { key: 'virtual' as const, label: 'Virtual', color: '#22d3ee' },
]

export function WorldCanvas() {
  const canvasRef = useRef<HTMLCanvasElement>(null)

  const fusedPose   = useRobotStore((s) => s.fusedPose)
  const fusedTrail  = useRobotStore((s) => s.fusedPoseTrail)
  const kinematics  = useRobotStore((s) => s.kinematics)
  const odomTrail   = useRobotStore((s) => s.odometryTrail)
  const gpsStatus     = useRobotStore((s) => s.gpsStatus)
  const tagDetections = useRobotStore((s) => s.tagDetections)
  const lidarPoints = useRobotStore((s) => s.lidarPoints)
  const obstacleTracks = useRobotStore((s) => s.obstacleTracks)
  const virtualTarget = useRobotStore((s) => s.virtualTarget)

  const [trails, setTrails] = useState<Trails>({ odom: true, gps: true, fused: true, lidar: true, virtual: true })

  const toggle = useCallback((key: keyof Trails) => {
    setTrails((t) => ({ ...t, [key]: !t[key] }))
  }, [])

  const fusedPoseRef   = useRef(fusedPose)
  const fusedTrailRef  = useRef(fusedTrail)
  const kinematicsRef  = useRef(kinematics)
  const odomTrailRef   = useRef(odomTrail)
  const gpsStatusRef   = useRef(gpsStatus)
  const tagDetectionsRef = useRef(tagDetections)
  const lidarPointsRef = useRef(lidarPoints)
  const obstacleTracksRef = useRef(obstacleTracks)
  const virtualTargetRef = useRef(virtualTarget)
  const trailsRef      = useRef(trails)

  useEffect(() => {
    fusedPoseRef.current = fusedPose
    fusedTrailRef.current = fusedTrail
    kinematicsRef.current = kinematics
    odomTrailRef.current = odomTrail
    gpsStatusRef.current = gpsStatus
    tagDetectionsRef.current = tagDetections
    lidarPointsRef.current = lidarPoints
    obstacleTracksRef.current = obstacleTracks
    virtualTargetRef.current = virtualTarget
    trailsRef.current = trails
  }, [fusedPose, fusedTrail, kinematics, odomTrail, gpsStatus, tagDetections, lidarPoints, obstacleTracks, virtualTarget, trails])

  useEffect(() => {
    let animId: number

    const tick = () => {
      const canvas = canvasRef.current
      if (!canvas) {
        animId = requestAnimationFrame(tick)
        return
      }

      const dpr = window.devicePixelRatio || 1
      const W   = canvas.clientWidth
      const H   = canvas.clientHeight
      if (!W || !H) {
        animId = requestAnimationFrame(tick)
        return
      }

      // Only resize the backing store if the layout size actually changed
      if (canvas.width !== W * dpr || canvas.height !== H * dpr) {
        canvas.width  = W * dpr
        canvas.height = H * dpr
      }

      const ctx = canvas.getContext('2d')
      if (!ctx) {
        animId = requestAnimationFrame(tick)
        return
      }

      // Reset transform and apply DPR scaling
      ctx.setTransform(1, 0, 0, 1, 0, 0)
      ctx.scale(dpr, dpr)

      const trailsVal = trailsRef.current
      const odomTrailVal = odomTrailRef.current
      const fusedTrailVal = fusedTrailRef.current
      const kinematicsVal = kinematicsRef.current
      const fusedPoseVal = fusedPoseRef.current
      const tagDetectionsVal = tagDetectionsRef.current
      const lidarPointsVal = lidarPointsRef.current
      const obstacleTracksVal = obstacleTracksRef.current
      const virtualTargetVal = virtualTargetRef.current

      // Calculate dynamic boundaries sequentially without array allocation or stack spread
      let minX = VENUE_EXT.minX
      let maxX = VENUE_EXT.maxX
      let minY = VENUE_EXT.minY
      let maxY = VENUE_EXT.maxY

      const updateBounds = (x: number, y: number) => {
        if (x - VENUE_PAD_MM < minX) minX = x - VENUE_PAD_MM
        if (x + VENUE_PAD_MM > maxX) maxX = x + VENUE_PAD_MM
        if (y - VENUE_PAD_MM < minY) minY = y - VENUE_PAD_MM
        if (y + VENUE_PAD_MM > maxY) maxY = y + VENUE_PAD_MM
      }

      if (trailsVal.fused) {
        for (let i = 0; i < fusedTrailVal.length; i++) {
          updateBounds(fusedTrailVal[i][0], fusedTrailVal[i][1])
        }
      }
      if (trailsVal.odom) {
        for (let i = 0; i < odomTrailVal.length; i++) {
          updateBounds(odomTrailVal[i][0], odomTrailVal[i][1])
        }
      }
      if (kinematicsVal) updateBounds(kinematicsVal.x, kinematicsVal.y)
      if (fusedPoseVal) updateBounds(fusedPoseVal.x, fusedPoseVal.y)
      if (trailsVal.gps) {
        for (let i = 0; i < tagDetectionsVal.length; i++) {
          updateBounds(tagDetectionsVal[i].x, tagDetectionsVal[i].y)
        }
      }
      if (trailsVal.virtual && virtualTargetVal) {
        updateBounds(virtualTargetVal.x, virtualTargetVal.y)
      }
      if (trailsVal.lidar) {
        for (let f = 0; f < lidarPointsVal.length; f++) {
          const frame = lidarPointsVal[f]
          for (let i = 0; i < frame.xs.length; i++) {
            updateBounds(frame.xs[i], frame.ys[i])
          }
        }
        for (let i = 0; i < obstacleTracksVal.length; i++) {
          const track = obstacleTracksVal[i]
          updateBounds(track.x - track.radius, track.y - track.radius)
          updateBounds(track.x + track.radius, track.y + track.radius)
        }
      }

      const rangeX = Math.max(maxX - minX, 1)
      const rangeY = Math.max(maxY - minY, 1)
      const scale  = Math.min(W / rangeX, H / rangeY)

      // Centre the content so padding is equal on all sides regardless of aspect ratio.
      const ox = (W - rangeX * scale) / 2
      const oy = (H - rangeY * scale) / 2

      const toC = (wx: number, wy: number): [number, number] => [
        ox + (wx - minX) * scale,
        oy + rangeY * scale - (wy - minY) * scale,
      ]

      // Translucent dark background
      ctx.fillStyle = 'rgba(0,0,0,0.55)'
      ctx.fillRect(0, 0, W, H)

      // Venue boundary rectangle
      {
        const [vx0, vy0] = toC(VENUE_LEFT_MM, VENUE_BOTTOM_MM)
        const [vx1, vy1] = toC(VENUE_RIGHT_MM, VENUE_TOP_MM)
        ctx.strokeStyle = 'rgba(255,255,255,0.18)'
        ctx.lineWidth   = 1
        ctx.strokeRect(vx0, vy1, vx1 - vx0, vy0 - vy1)
      }

      // Venue grid lines at 610 mm
      ctx.strokeStyle = 'rgba(255,255,255,0.08)'
      ctx.lineWidth   = 0.5
      for (let col = 0; col <= VENUE_COLS; col++) {
        const [cx] = toC(VENUE_LEFT_MM + col * GRID_MM, VENUE_BOTTOM_MM)
        ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, H); ctx.stroke()
      }
      for (let row = 0; row <= VENUE_ROWS; row++) {
        const [, cy] = toC(0, VENUE_BOTTOM_MM + row * GRID_MM)
        ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(W, cy); ctx.stroke()
      }

      // Origin cross
      const [originX, originY] = toC(0, 0)
      if (originX >= 0 && originX <= W && originY >= 0 && originY <= H) {
        ctx.strokeStyle = 'rgba(255,255,255,0.20)'
        ctx.lineWidth = 1
        ctx.beginPath(); ctx.moveTo(originX - 6, originY); ctx.lineTo(originX + 6, originY); ctx.stroke()
        ctx.beginPath(); ctx.moveTo(originX, originY - 6); ctx.lineTo(originX, originY + 6); ctx.stroke()
      }

      // Scale label
      ctx.fillStyle = 'rgba(255,255,255,0.40)'
      ctx.font = '9px monospace'
      ctx.textAlign = 'right'
      ctx.textBaseline = 'bottom'
      ctx.fillText(`grid: ${GRID_MM} mm`, W - 4, H - 3)

      // Odometry trail (blue)
      if (trailsVal.odom && odomTrailVal.length > 1) {
        ctx.beginPath()
        ctx.strokeStyle = 'rgba(96,165,250,0.55)'
        ctx.lineWidth = 1.5
        ctx.lineJoin = 'round'
        odomTrailVal.forEach(([wx, wy], i) => {
          const [cx, cy] = toC(wx, wy)
          i === 0 ? ctx.moveTo(cx, cy) : ctx.lineTo(cx, cy)
        })
        ctx.stroke()
      }
      if (trailsVal.odom && odomTrailVal.length === 1) {
        const [cx, cy] = toC(odomTrailVal[0][0], odomTrailVal[0][1])
        ctx.beginPath()
        ctx.arc(cx, cy, 2.5, 0, Math.PI * 2)
        ctx.fillStyle = 'rgba(96,165,250,0.80)'
        ctx.fill()
      }

      // Fused trail (green)
      if (trailsVal.fused && fusedTrailVal.length > 1) {
        ctx.beginPath()
        ctx.strokeStyle = 'rgba(74,222,128,0.70)'
        ctx.lineWidth = 1.5
        ctx.lineJoin = 'round'
        fusedTrailVal.forEach(([wx, wy], i) => {
          const [cx, cy] = toC(wx, wy)
          i === 0 ? ctx.moveTo(cx, cy) : ctx.lineTo(cx, cy)
        })
        ctx.stroke()
      }

      // GPS tags — yellow cross + ID label
      if (trailsVal.gps && tagDetectionsVal.length > 0) {
        const arm = 6
        ctx.strokeStyle = 'rgba(250,204,21,0.90)'
        ctx.lineWidth = 2
        ctx.fillStyle = 'rgba(250,204,21,0.90)'
        ctx.font = '10px monospace'
        ctx.textAlign = 'left'
        ctx.textBaseline = 'bottom'
        for (let i = 0; i < tagDetectionsVal.length; i++) {
          const tag = tagDetectionsVal[i]
          const [cx, cy] = toC(tag.x, tag.y)
          ctx.beginPath(); ctx.moveTo(cx - arm, cy); ctx.lineTo(cx + arm, cy); ctx.stroke()
          ctx.beginPath(); ctx.moveTo(cx, cy - arm); ctx.lineTo(cx, cy + arm); ctx.stroke()
          ctx.fillText(`#${tag.tag_id}`, cx + arm + 2, cy)
        }
      }

      // Lidar cloud — fast pixel blitting instead of complex circles
      if (trailsVal.lidar && lidarPointsVal.length > 0) {
        for (let f = 0; f < lidarPointsVal.length; f++) {
          const alpha = 0.25 + 0.45 * (f / (LIDAR_WINDOW_FRAMES - 1 || 1))
          ctx.fillStyle = `rgba(248,113,113,${alpha.toFixed(2)})`
          const frame = lidarPointsVal[f]
          for (let i = 0; i < frame.xs.length; i++) {
            const [cx, cy] = toC(frame.xs[i], frame.ys[i])
            ctx.fillRect(cx - 1, cy - 1, 2, 2)
          }
        }
      }

      // Obstacle tracks
      if (trailsVal.lidar && obstacleTracksVal.length > 0) {
        ctx.strokeStyle = 'rgba(248,113,113,0.95)'
        ctx.fillStyle = 'rgba(248,113,113,0.95)'
        ctx.lineWidth = 1.5
        ctx.font = '10px monospace'
        ctx.textAlign = 'left'
        ctx.textBaseline = 'bottom'
        for (let i = 0; i < obstacleTracksVal.length; i++) {
          const track = obstacleTracksVal[i]
          const [cx, cy] = toC(track.x, track.y)
          const radiusPx = Math.max(2, track.radius * scale)
          ctx.beginPath()
          ctx.arc(cx, cy, radiusPx, 0, Math.PI * 2)
          ctx.stroke()
          ctx.fillText(`#${track.id}`, cx + radiusPx + 3, cy - 2)
        }
      }

      // Virtual target
      if (trailsVal.virtual && virtualTargetVal) {
        const [cx, cy] = toC(virtualTargetVal.x, virtualTargetVal.y)
        ctx.fillStyle = 'rgba(34,211,238,0.95)'
        ctx.strokeStyle = 'rgba(34,211,238,0.95)'
        ctx.lineWidth = 1.5
        ctx.beginPath()
        ctx.arc(cx, cy, 4.0, 0, Math.PI * 2)
        ctx.fill()
        ctx.font = '10px monospace'
        ctx.textAlign = 'left'
        ctx.textBaseline = 'bottom'
        ctx.fillText('LVT', cx + 6, cy - 2)
      }

      const drawRobot = (
        x: number,
        y: number,
        theta: number,
        stroke: string,
        fill: string,
      ) => {
        const [rx, ry] = toC(x, y)
        const headingCanvas = -theta  // world CCW → canvas CW
        const arrowLen = Math.max(12, Math.min(28, Math.min(rangeX, rangeY) * scale * 0.06))
        const nx = Math.cos(headingCanvas), ny = Math.sin(headingCanvas)
        const tipX = rx + nx * arrowLen, tipY = ry + ny * arrowLen
        const hw = 4

        ctx.beginPath()
        ctx.arc(rx, ry, 7, 0, Math.PI * 2)
        ctx.fillStyle   = fill
        ctx.strokeStyle = stroke
        ctx.lineWidth   = 1.5
        ctx.fill()
        ctx.stroke()

        ctx.strokeStyle = stroke
        ctx.lineWidth   = 2
        ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(tipX, tipY); ctx.stroke()

        ctx.fillStyle = stroke
        ctx.beginPath()
        ctx.moveTo(tipX, tipY)
        ctx.lineTo(tipX - nx * 8 + ny * hw, tipY - ny * 8 - nx * hw)
        ctx.lineTo(tipX - nx * 8 - ny * hw, tipY - ny * 8 + nx * hw)
        ctx.closePath()
        ctx.fill()
      }

      // Robot at fused pose when available; otherwise fall back to raw odometry.
      if (fusedPoseVal) {
        drawRobot(
          fusedPoseVal.x,
          fusedPoseVal.y,
          fusedPoseVal.theta,
          '#4ade80',
          'rgba(74,222,128,0.25)',
        )
      } else if (kinematicsVal) {
        drawRobot(
          kinematicsVal.x,
          kinematicsVal.y,
          kinematicsVal.theta,
          '#60a5fa',
          'rgba(96,165,250,0.25)',
        )
      }

      animId = requestAnimationFrame(tick)
    }

    animId = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(animId)
  }, [])

  return (
    <div className="relative rounded-2xl p-4 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl">
      <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent rounded-t-2xl" />
      <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50" />

      <div className="relative">
        <h4 className="text-sm font-semibold text-white mb-3">World Map</h4>

        {/* Series toggles — DCPlot legend style */}
        <div className="flex flex-wrap gap-x-3 gap-y-1 mb-2">
          {SERIES.map(({ key, label, color }) => (
            <button
              key={key}
              onClick={() => toggle(key)}
              className="flex items-center gap-1.5 cursor-pointer rounded px-1 py-0.5 transition-opacity hover:bg-white/10"
              style={{ opacity: trails[key] ? 1 : 0.35 }}
            >
              <span style={{ display: 'inline-block', width: 12, height: 3, borderRadius: 2, background: color }} />
              <span className="text-xs font-medium text-white/80">{label}</span>
            </button>
          ))}
        </div>

        <canvas
          ref={canvasRef}
          className="w-full rounded-xl"
          style={{ aspectRatio: `${MAP_WIDTH_MM} / ${MAP_HEIGHT_MM}` }}
        />
      </div>
    </div>
  )
}
