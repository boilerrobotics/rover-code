import { ComponentFixture, TestBed } from '@angular/core/testing';

import { MiniRoverComponent } from './mini-rover.component';

describe('MiniRoverComponent', () => {
  let component: MiniRoverComponent;
  let fixture: ComponentFixture<MiniRoverComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ MiniRoverComponent ]
    })
    .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(MiniRoverComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
