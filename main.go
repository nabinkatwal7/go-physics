package main

import (
	"image/color"
	"math"
	"math/rand"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

const (
	screenWidth  = 800
	screenHeight = 600
	particleSize = 10
)

// Vector2 represents a 2D vector
type Vector2 struct {
	X, Y float64
}

func (v Vector2) Add(other Vector2) Vector2 {
	return Vector2{v.X + other.X, v.Y + other.Y}
}

func (v Vector2) Subtract(other Vector2) Vector2 {
	return Vector2{v.X - other.X, v.Y - other.Y}
}

func (v Vector2) Multiply(scalar float64) Vector2 {
	return Vector2{v.X * scalar, v.Y * scalar}
}

func (v Vector2) Magnitude() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y)
}

func (v Vector2) Normalize() Vector2 {
	mag := v.Magnitude()
	if mag == 0 {
		return Vector2{0, 0}
	}
	return Vector2{v.X / mag, v.Y / mag}
}

// Particle represents a physical object
type Particle struct {
	Position     Vector2
	Velocity     Vector2
	Acceleration Vector2
	Mass         float64
	Radius       float64
	Color        color.RGBA
	Fixed        bool
}

func NewParticle(x, y float64) *Particle {
	return &Particle{
		Position: Vector2{x, y},
		Velocity: Vector2{rand.Float64()*2 - 1, rand.Float64()*2 - 1},
		Mass:     rand.Float64()*2 + 0.5,
		Radius:   float64(particleSize),
		Color: color.RGBA{
			R: uint8(rand.Intn(200) + 55),
			G: uint8(rand.Intn(200) + 55),
			B: uint8(rand.Intn(200) + 55),
			A: 255,
		},
	}
}

func (p *Particle) ApplyForce(force Vector2) {
	if p.Fixed {
		return
	}
	p.Acceleration = p.Acceleration.Add(force.Multiply(1 / p.Mass))
}

func (p *Particle) Update(dt float64) {
	if p.Fixed {
		return
	}

	// Semi-implicit Euler integration
	p.Velocity = p.Velocity.Add(p.Acceleration.Multiply(dt))
	p.Position = p.Position.Add(p.Velocity.Multiply(dt))
	
	p.Acceleration = Vector2{0, 0}
}

// PhysicsEngine manages the simulation
type PhysicsEngine struct {
	Particles []*Particle
	Gravity   Vector2
}

func NewPhysicsEngine() *PhysicsEngine {
	pe := &PhysicsEngine{
		Gravity: Vector2{0, 98}, // Stronger gravity for better visual effect
	}

	// Add fixed particles as boundaries
	pe.addBoundaries()

	return pe
}

func (pe *PhysicsEngine) AddParticle(x, y float64) {
	pe.Particles = append(pe.Particles, NewParticle(x, y))
}

func (pe *PhysicsEngine) addBoundaries() {
	// Bottom boundary
	for x := 0.0; x < float64(screenWidth); x += float64(particleSize) * 2 {
		p := NewParticle(x, float64(screenHeight)-float64(particleSize))
		p.Fixed = true
		p.Color = color.RGBA{100, 100, 100, 255}
		pe.Particles = append(pe.Particles, p)
	}

	// Left and right boundaries
	for y := float64(screenHeight) / 2; y < float64(screenHeight); y += float64(particleSize) * 2 {
		// Left
		p := NewParticle(float64(particleSize), y)
		p.Fixed = true
		p.Color = color.RGBA{100, 100, 100, 255}
		pe.Particles = append(pe.Particles, p)

		// Right
		p = NewParticle(float64(screenWidth)-float64(particleSize), y)
		p.Fixed = true
		p.Color = color.RGBA{100, 100, 100, 255}
		pe.Particles = append(pe.Particles, p)
	}
}

func (pe *PhysicsEngine) Update(dt float64) {
	// Apply forces
	for _, p := range pe.Particles {
		p.ApplyForce(pe.Gravity.Multiply(p.Mass))
	}

	// Handle collisions
	pe.handleCollisions()

	// Update particles
	for _, p := range pe.Particles {
		p.Update(dt)
		pe.applyWorldBounds(p)
	}
}

func (pe *PhysicsEngine) applyWorldBounds(p *Particle) {
	if p.Position.X-p.Radius < 0 {
		p.Position.X = p.Radius
		p.Velocity.X *= -0.8
	}
	if p.Position.X+p.Radius > float64(screenWidth) {
		p.Position.X = float64(screenWidth) - p.Radius
		p.Velocity.X *= -0.8
	}
	if p.Position.Y-p.Radius < 0 {
		p.Position.Y = p.Radius
		p.Velocity.Y *= -0.8
	}
}

func (pe *PhysicsEngine) handleCollisions() {
	for i := 0; i < len(pe.Particles); i++ {
		for j := i + 1; j < len(pe.Particles); j++ {
			p1 := pe.Particles[i]
			p2 := pe.Particles[j]

			distVec := p1.Position.Subtract(p2.Position)
			distance := distVec.Magnitude()
			minDistance := p1.Radius + p2.Radius

			if distance < minDistance {
				pe.resolveCollision(p1, p2, distVec.Normalize(), distance, minDistance)
			}
		}
	}
}

func (pe *PhysicsEngine) resolveCollision(p1, p2 *Particle, normal Vector2, distance, minDistance float64) {
	penetration := minDistance - distance

	if !p1.Fixed && !p2.Fixed {
		correction := normal.Multiply(penetration * 0.5)
		p1.Position = p1.Position.Add(correction)
		p2.Position = p2.Position.Subtract(correction)
	} else if !p1.Fixed {
		p1.Position = p1.Position.Add(normal.Multiply(penetration))
	} else if !p2.Fixed {
		p2.Position = p2.Position.Subtract(normal.Multiply(penetration))
	}

	relativeVelocity := p1.Velocity.Subtract(p2.Velocity)
	velocityAlongNormal := relativeVelocity.X*normal.X + relativeVelocity.Y*normal.Y

	if velocityAlongNormal > 0 {
		return
	}

	restitution := 0.7
	j := -(1 + restitution) * velocityAlongNormal
	j /= 1/p1.Mass + 1/p2.Mass

	impulse := normal.Multiply(j)
	if !p1.Fixed {
		p1.Velocity = p1.Velocity.Add(impulse.Multiply(1 / p1.Mass))
	}
	if !p2.Fixed {
		p2.Velocity = p2.Velocity.Subtract(impulse.Multiply(1 / p2.Mass))
	}
}

func (pe *PhysicsEngine) Draw(screen *ebiten.Image) {
	for _, p := range pe.Particles {
		ebitenutil.DrawCircle(
			screen,
			p.Position.X,
			p.Position.Y,
			p.Radius,
			p.Color,
		)
	}
}

// Game implements ebiten.Game interface
type Game struct {
	Physics *PhysicsEngine
}

func (g *Game) Update() error {
	dt := 1.0 / 60.0 // Assuming 60 FPS
	
	// Add new particle on mouse click
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()
		g.Physics.AddParticle(float64(x), float64(y))
	}
	
	g.Physics.Update(dt)
	return nil
}

func (g *Game) Draw(screen *ebiten.Image) {
	screen.Fill(color.RGBA{20, 20, 35, 255})
	g.Physics.Draw(screen)
	
	// Draw instructions
	ebitenutil.DebugPrint(screen, "Click to add particles")
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	return screenWidth, screenHeight
}

func main() {
	ebiten.SetWindowSize(screenWidth, screenHeight)
	ebiten.SetWindowTitle("Physics Engine with Mouse Interaction")

	game := &Game{
		Physics: NewPhysicsEngine(),
	}

	if err := ebiten.RunGame(game); err != nil {
		panic(err)
	}
}